/**
 * @file robotic_total_station_error.hpp
 * @brief Header file for the GpError class. Reference: imu_error.cpp
 * @author Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
 */

#include "svo/ceres_backend/global_position_error.hpp"

#include <svo/vio_common/matrix.hpp>
#include <svo/vio_common/matrix_operations.hpp>
#include <svo/common/conversions.h>

#include "svo/ceres_backend/pose_local_parameterization.hpp"

namespace svo {
namespace ceres_backend {

// Construct with measurements and parameters.
GpError::GpError(const GpMeasurements &gp_measurements,
                   const GpParameters& gp_parameters,
                   const ImuMeasurements &imu_measurements,
                   const ImuParameters& imu_parameters,
                   const double& t_0, const double& t_1)
{
  // Functions inherited from ceres::CostFunction.
  // https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/cost_function.h
  // Set int num_residuals_.
  int number_of_residuals = 3*gp_measurements.size();
  set_num_residuals(number_of_residuals);

  // Push back in std::vector<int32_t> parameter_block_sizes_
  // size of first parameter (PoseParameterBlock k)
  mutable_parameter_block_sizes()->push_back(7);
  // size of second parameter (SpeedAndBiasParameterBlock k)
  mutable_parameter_block_sizes()->push_back(9);

  std::lock_guard<std::mutex> lock(preintegration_mutex_);
  setGpMeasurements(gp_measurements);
  setGpParameters(gp_parameters);
  setImuMeasurements(imu_measurements);
  setImuParameters(imu_parameters);
  setT0(t_0 - imu_parameters.delay_imu_cam);
  setT1(t_1);;

  setGpCovariance();

  DEBUG_CHECK(imu_measurements.back().timestamp_ < gp_measurements.back().timestamp_)
      << "Oldest IMU measurement is newer than oldest gp measurement!";
  DEBUG_CHECK(imu_measurements.front().timestamp_ > gp_measurements.front().timestamp_)
      << "Newest IMU measurement is older than newest gp measurement!";
}

// Not used.
void GpError::setGpImuIndexCorrespondences()
{
    std::size_t cnt = 0;
    for (size_t i = 0; i < gp_measurements_.size(); ++i)
    {
      for (size_t j = cnt; j < imu_measurements_.size(); ++j)
      {
        if (imu_measurements_[j].timestamp_ < gp_measurements_[i].timestamp_)
        {
          index_first_imu_meas_next_in_time_.push_back(j-1);
          cnt = j;
          break;
        }
      }
    }
}

// Propagates pose, speeds and biases with given IMU measurements.
int GpError::propagation(const ImuMeasurements &imu_measurements,
                          const ImuParameters& imu_parameters,
                          const GpMeasurements &gp_measurements,
                          Transformation& T_WS0,
                          Transformations &T_WS,
                          SpeedAndBias& speed_and_biases0,
                          SpeedsAndBiases &speed_and_biases,
                          const double &t_start,
                          const bool compute_covariance,
                          const bool compute_jacobian,
                          Covariances &covariances,
                          Jacobians &jacobians)
{
  // now the propagation
  double time = t_start;

  // sanity check
  assert(gp_measurements.back().timestamp_ > time);

  // Oldest imu meas older than oldest gp meas.
  if (!(imu_measurements.back().timestamp_ < gp_measurements.back().timestamp_))
  {
    return -1;  // nothing to do...
  }

  // Newest imu meas newer than newest gp meas.
  if (!(imu_measurements.front().timestamp_ > gp_measurements.front().timestamp_))
  {
    return -1;  // nothing to do...
  }

  // initial condition
  Eigen::Vector3d r_0 = T_WS0.getPosition();
  Eigen::Quaterniond q_WS_0 = T_WS0.getEigenQuaternion();
  Eigen::Matrix3d C_WS_0 = T_WS0.getRotationMatrix();

  // increments
  Eigen::Quaterniond Delta_q = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  Eigen::Matrix<double,15,15> P_delta = Eigen::Matrix<double,15,15>::Zero();

  // Vectors for preintegrated values at rts residual time.
  std::vector<Eigen::Quaterniond> Delta_q_vec;
  std::vector<Eigen::Matrix3d> C_doubleintegral_vec;
  std::vector<Eigen::Matrix3d> C_integral_vec;
  std::vector<Eigen::Vector3d> acc_integral_vec;
  std::vector<Eigen::Vector3d> acc_doubleintegral_vec;
  std::vector<Eigen::Matrix3d> dalpha_db_g_vec;
  std::vector<Eigen::Matrix3d> dp_db_g_vec;
  std::vector<Eigen::Matrix3d> dv_db_g_vec;
  std::vector<Eigen::Matrix<double, 15, 15>> P_delta_at_t_rts_vec;

  bool has_started = false;
  bool last_iteration = false;
  int num_propagated = 0;
  size_t n_gp_meas = gp_measurements.size();
  size_t n_to_integrate_at_t_rts = n_gp_meas;
  for (size_t i = imu_measurements.size()-1; i != 0u; --i)
  {
    Eigen::Vector3d omega_S_0 = imu_measurements[i].angular_velocity_;
    Eigen::Vector3d acc_S_0 = imu_measurements[i].linear_acceleration_;
    Eigen::Vector3d omega_S_1 = imu_measurements[i-1].angular_velocity_;
    Eigen::Vector3d acc_S_1 = imu_measurements[i-1].linear_acceleration_;
    double nexttime = imu_measurements[i - 1].timestamp_;

    // time delta
    double dt = nexttime - time;

    // Sanity check
    if (dt <= 0.0)
    {
      continue;
    }

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu_measurements[i].timestamp_);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_parameters.sigma_g_c;
    double sigma_a_c = imu_parameters.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_parameters.g_max
        || std::abs(omega_S_0[1]) > imu_parameters.g_max
        || std::abs(omega_S_0[2]) > imu_parameters.g_max)
    {
      sigma_g_c *= 100;
      LOG(WARNING)<< "gyr saturation";
    }

    if (std::abs(acc_S_0[0]) > imu_parameters.a_max
        || std::abs(acc_S_0[1]) > imu_parameters.a_max
        || std::abs(acc_S_0[2]) > imu_parameters.a_max)
    {
      sigma_a_c *= 100;
      LOG(WARNING)<< "acc saturation";
    }

    // Get propagation terms at residual time
    if (gp_measurements[n_to_integrate_at_t_rts - 1].timestamp_ <= nexttime)
    {
      n_to_integrate_at_t_rts--;

      if (n_to_integrate_at_t_rts == 0)
      {
        last_iteration = true;
      }

      // Interpolate
      double interval = nexttime - imu_measurements[i].timestamp_;
      double dt_till_t_rts = gp_measurements[n_to_integrate_at_t_rts].timestamp_ - time;
      const double r = dt_till_t_rts / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();

      // ensure integrity
      if (std::abs(omega_S_1[0]) > imu_parameters.g_max
          || std::abs(omega_S_1[1]) > imu_parameters.g_max
          || std::abs(omega_S_1[2]) > imu_parameters.g_max)
      {
        sigma_g_c *= 100;
        LOG(WARNING)<< "gyr saturation";
      }

      if (std::abs(acc_S_1[0]) > imu_parameters.a_max
          || std::abs(acc_S_1[1]) > imu_parameters.a_max
          || std::abs(acc_S_1[2]) > imu_parameters.a_max)
      {
        sigma_a_c *= 100;
        LOG(WARNING)<< "acc saturation";
      }

      // actual propagation
      // orientation:
      Eigen::Quaterniond dq;
      const Eigen::Vector3d omega_S_true =
          (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases0.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt_till_t_rts;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt_till_t_rts;
      dq.w() = cos_theta_half;
      Eigen::Quaterniond Delta_q_1 = Delta_q * dq;
      // rotation matrix integral:
      const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
      const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
      const Eigen::Vector3d acc_S_true =
          (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases0.segment<3>(6));
      const Eigen::Matrix3d C_integral_1 = C_integral + 0.5 * (C + C_1) * dt_till_t_rts;
      const Eigen::Vector3d acc_integral_1 =
          acc_integral + 0.5 * (C + C_1) * acc_S_true * dt_till_t_rts;
      // rotation matrix double integral:
      const Eigen::Matrix3d C_doubleintegral_at_t_rts = C_doubleintegral +
              C_integral * dt_till_t_rts + 0.25 * (C + C_1) * dt_till_t_rts * dt_till_t_rts;
      const Eigen::Vector3d acc_doubleintegral_at_t_rts = acc_doubleintegral +
              acc_integral * dt_till_t_rts
              + 0.25 * (C + C_1) * acc_S_true * dt_till_t_rts * dt_till_t_rts;

      // Jacobian parts
      const Eigen::Matrix3d dalpha_db_g_at_t_rts = dalpha_db_g
              + dt_till_t_rts * C_1;
      const Eigen::Matrix3d cross_1 =
          dq.inverse().toRotationMatrix() * cross
          + expmapDerivativeSO3(omega_S_true * dt_till_t_rts) * dt_till_t_rts;
      const Eigen::Matrix3d acc_S_x = skewSymmetric(acc_S_true);
      Eigen::Matrix3d dv_db_g_1 =
          dv_db_g + 0.5 * dt_till_t_rts * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      const Eigen::Matrix3d dp_db_g_at_t_rts = dp_db_g +
              dt_till_t_rts * dv_db_g
              + 0.25 * dt_till_t_rts * dt_till_t_rts
              * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);

      // covariance propagation
      if (compute_covariance)
      {
        Eigen::Matrix<double, 15, 15> F_delta =
                Eigen::Matrix<double, 15, 15>::Identity();
        // transform
        F_delta.block<3, 3>(0, 3) =
                -skewSymmetric(acc_integral * dt_till_t_rts
                               + 0.25 * (C + C_1) * acc_S_true * dt_till_t_rts * dt_till_t_rts);
        F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt_till_t_rts;
        F_delta.block<3, 3>(0, 9) =
                dt_till_t_rts * dv_db_g
                + 0.25 * dt_till_t_rts * dt_till_t_rts
                * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(0, 12) = -C_integral * dt_till_t_rts
                + 0.25 * (C + C_1) * dt_till_t_rts * dt_till_t_rts;
        F_delta.block<3, 3>(3, 9) = -dt_till_t_rts * C_1;
        F_delta.block<3, 3>(6, 3) =
                -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt_till_t_rts);
        F_delta.block<3, 3>(6, 9) =
                0.5 * dt_till_t_rts * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt_till_t_rts;

        Eigen::Matrix<double, 15, 15> P_delta_at_t_rts =
                Eigen::Matrix<double, 15, 15>::Identity();;
        P_delta_at_t_rts = F_delta * P_delta * F_delta.transpose();
        // add noise. Note that transformations with rotation matrices can be
        // ignored, since the noise is isotropic.
        //F_tot = F_delta*F_tot;
        const double sigma2_dalpha = dt_till_t_rts * sigma_g_c * sigma_g_c;
        P_delta_at_t_rts(3, 3) += sigma2_dalpha;
        P_delta_at_t_rts(4, 4) += sigma2_dalpha;
        P_delta_at_t_rts(5, 5) += sigma2_dalpha;
        const double sigma2_v = dt_till_t_rts * sigma_a_c * sigma_a_c;
        P_delta_at_t_rts(6, 6) += sigma2_v;
        P_delta_at_t_rts(7, 7) += sigma2_v;
        P_delta_at_t_rts(8, 8) += sigma2_v;
        const double sigma2_p = 0.5 * dt_till_t_rts * dt_till_t_rts * sigma2_v;
        P_delta_at_t_rts(0, 0) += sigma2_p;
        P_delta_at_t_rts(1, 1) += sigma2_p;
        P_delta_at_t_rts(2, 2) += sigma2_p;
        const double sigma2_b_g =
                dt_till_t_rts * imu_parameters.sigma_gw_c * imu_parameters.sigma_gw_c;
        P_delta_at_t_rts(9, 9) += sigma2_b_g;
        P_delta_at_t_rts(10, 10) += sigma2_b_g;
        P_delta_at_t_rts(11, 11) += sigma2_b_g;
        const double sigma2_b_a =
                dt_till_t_rts * imu_parameters.sigma_aw_c* imu_parameters.sigma_aw_c;
        P_delta_at_t_rts(12, 12) += sigma2_b_a;
        P_delta_at_t_rts(13, 13) += sigma2_b_a;
        P_delta_at_t_rts(14, 14) += sigma2_b_a;

        // store quantity
        P_delta_at_t_rts_vec.push_back(P_delta_at_t_rts);
      }

      // store quantities
      Delta_q_vec.push_back(Delta_q_1);
      C_doubleintegral_vec.push_back(C_doubleintegral_at_t_rts);
      C_integral_vec.push_back(C_integral_1);
      acc_integral_vec.push_back(acc_integral_1);
      acc_doubleintegral_vec.push_back(acc_doubleintegral_at_t_rts);
      dalpha_db_g_vec.push_back(dalpha_db_g_at_t_rts);
      dp_db_g_vec.push_back(dp_db_g_at_t_rts);
      dv_db_g_vec.push_back(dv_db_g_1);
    }

    if (last_iteration)
    {
      break;
    }
    else
    {
      // ensure integrity
      if (std::abs(omega_S_1[0]) > imu_parameters.g_max
          || std::abs(omega_S_1[1]) > imu_parameters.g_max
          || std::abs(omega_S_1[2]) > imu_parameters.g_max)
      {
        sigma_g_c *= 100;
        LOG(WARNING)<< "gyr saturation";
      }

      if (std::abs(acc_S_1[0]) > imu_parameters.a_max
          || std::abs(acc_S_1[1]) > imu_parameters.a_max
          || std::abs(acc_S_1[2]) > imu_parameters.a_max)
      {
        sigma_a_c *= 100;
        LOG(WARNING)<< "acc saturation";
      }

      // actual propagation
      // orientation:
      Eigen::Quaterniond dq;
      const Eigen::Vector3d omega_S_true =
              (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases0.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
      dq.w() = cos_theta_half;
      Eigen::Quaterniond Delta_q_1 = Delta_q * dq;
      // rotation matrix integral:
      const Eigen::Matrix3d C = Delta_q.toRotationMatrix();
      const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
      const Eigen::Vector3d acc_S_true =
              (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases0.segment<3>(6));
      const Eigen::Matrix3d C_integral_1 = C_integral + 0.5 * (C + C_1) * dt;
      const Eigen::Vector3d acc_integral_1 =
              acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
      // rotation matrix double integral:
      C_doubleintegral += C_integral * dt + 0.25 * (C + C_1) * dt * dt;
      acc_doubleintegral +=
              acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

      // Jacobian parts
      //dalpha_db_g += C_1 * expmapDerivativeSO3(omega_S_true * dt) * dt;
      dalpha_db_g += dt * C_1;
      const Eigen::Matrix3d cross_1 =
              dq.inverse().toRotationMatrix() * cross
              + expmapDerivativeSO3(omega_S_true * dt) * dt;
      const Eigen::Matrix3d acc_S_x = skewSymmetric(acc_S_true);
      Eigen::Matrix3d dv_db_g_1 =
              dv_db_g + 0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
      dp_db_g += dt * dv_db_g
              + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);

      if (compute_covariance)
      {
        // covariance propagation
        Eigen::Matrix<double, 15, 15> F_delta =
            Eigen::Matrix<double, 15, 15>::Identity();
        // transform
        F_delta.block<3, 3>(0, 3) =
            -skewSymmetric(acc_integral * dt
                           + 0.25 * (C + C_1) * acc_S_true * dt * dt);
        F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
        F_delta.block<3, 3>(0, 9) =
            dt * dv_db_g
            + 0.25 * dt * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(0, 12) = -C_integral * dt + 0.25 * (C + C_1) * dt * dt;
        F_delta.block<3, 3>(3, 9) = -dt * C_1;
        F_delta.block<3, 3>(6, 3) =
            -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt);
        F_delta.block<3, 3>(6, 9) =
            0.5 * dt * (C * acc_S_x * cross + C_1 * acc_S_x * cross_1);
        F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
        P_delta = F_delta * P_delta * F_delta.transpose();
        // add noise. Note that transformations with rotation matrices can be
        // ignored, since the noise is isotropic.
        //F_tot = F_delta*F_tot;
        const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
        P_delta(3, 3) += sigma2_dalpha;
        P_delta(4, 4) += sigma2_dalpha;
        P_delta(5, 5) += sigma2_dalpha;
        const double sigma2_v = dt * sigma_a_c * sigma_a_c;
        P_delta(6, 6) += sigma2_v;
        P_delta(7, 7) += sigma2_v;
        P_delta(8, 8) += sigma2_v;
        const double sigma2_p = 0.5 * dt * dt * sigma2_v;
        P_delta(0, 0) += sigma2_p;
        P_delta(1, 1) += sigma2_p;
        P_delta(2, 2) += sigma2_p;
        const double sigma2_b_g =
            dt * imu_parameters.sigma_gw_c * imu_parameters.sigma_gw_c;
        P_delta(9, 9) += sigma2_b_g;
        P_delta(10, 10) += sigma2_b_g;
        P_delta(11, 11) += sigma2_b_g;
        const double sigma2_b_a =
            dt * imu_parameters.sigma_aw_c* imu_parameters.sigma_aw_c;
        P_delta(12, 12) += sigma2_b_a;
        P_delta(13, 13) += sigma2_b_a;
        P_delta(14, 14) += sigma2_b_a;
      }

      // memory shift
      Delta_q = Delta_q_1;
      C_integral = C_integral_1;
      acc_integral = acc_integral_1;
      cross = cross_1;
      dv_db_g = dv_db_g_1;
      time = nexttime;

      ++num_propagated;
    }

  }

  // actual propagation output:
  const Eigen::Vector3d g_W = imu_parameters.g * Eigen::Vector3d(0, 0, 1.0);
  for (size_t i = 0; i < n_gp_meas; i++)
  {
    size_t ind_rts = n_gp_meas-i-1;
    double Delta_t = gp_measurements[ind_rts].timestamp_ - t_start;
    T_WS.push_back(
        Transformation(
                    r_0 + speed_and_biases0.head<3>() * Delta_t
                    + C_WS_0 * (acc_doubleintegral_vec[i]/*-C_doubleintegral*speedAndBiases.segment<3>(6)*/)
                    - 0.5 * g_W * Delta_t * Delta_t,
                    q_WS_0 * Delta_q_vec[i])
                );

    speed_and_biases.push_back(speed_and_biases0);

    speed_and_biases[i].head<3>() += C_WS_0 * (acc_integral_vec[i]/*-C_integral*speedAndBiases.segment<3>(6)*/)
            - g_W * Delta_t;
  }


  // assign Jacobian, if requested (Only at ;ast rts measurement at the moment)
  if (compute_jacobian)
  {
      for (size_t i = 0; i < n_gp_meas; ++i)
      {
        size_t ind_rts = n_gp_meas-i-1;
        double Delta_t = gp_measurements[ind_rts].timestamp_ - t_start;

        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();
        F.block<3,3>(0,3) = -skewSymmetric(C_WS_0 * acc_doubleintegral_vec[i]);
        F.block<3,3>(0,6) = Eigen::Matrix3d::Identity() * Delta_t;
        F.block<3,3>(0,9) = C_WS_0 * dp_db_g_vec[i];
        F.block<3,3>(0,12) = -C_WS_0 * C_doubleintegral_vec[i];
        F.block<3,3>(3,9) = -C_WS_0 * dalpha_db_g_vec[i];
        F.block<3,3>(6,3) = -skewSymmetric(C_WS_0 * acc_integral_vec[i]);
        F.block<3,3>(6,9) = C_WS_0 * dv_db_g_vec[i];
        F.block<3,3>(6,12) = -C_WS_0 * C_integral_vec[i];

        jacobians.push_back(F);

      }
  }

  // overall covariance, if requested (Only at ;ast rts measurement at the moment)
  if (compute_covariance)
  {
    for (size_t i = 0; i < n_gp_meas; ++i)
    {
      Eigen::Matrix<double,15,15> P;
      // transform from local increments to actual states
      Eigen::Matrix<double,15,15> T = Eigen::Matrix<double,15,15>::Identity();
      T.topLeftCorner<3,3>() = C_WS_0;
      T.block<3,3>(3,3) = C_WS_0;
      T.block<3,3>(6,6) = C_WS_0;
      P = T * P_delta_at_t_rts_vec[i] * T.transpose();

      covariances.push_back(P);
    }
  }

  return num_propagated;
}

// Propagates pose, speeds and biases with given IMU measurements.
int GpError::redoPreintegration(const Transformation& /*T_WS*/,
                                 const SpeedAndBias& speed_and_biases) const
{ 
  // ensure unique access
  std::lock_guard<std::mutex> lock(preintegration_mutex_);

  // now the propagation
  double time = t0_;

  // sanity check
  assert(gp_measurements_.back().timestamp_ >= time);

  // Oldest imu meas older than oldest gp meas.
  if (!(imu_measurements_.back().timestamp_ < gp_measurements_.back().timestamp_))
  {
    return -1;  // nothing to do...
  }

  // Newest imu meas newer than newest gp meas.
  if (!(imu_measurements_.front().timestamp_ > gp_measurements_.front().timestamp_))
  {
    return -1;  // nothing to do...
  }

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  C_doubleintegral_ = Eigen::Matrix3d::Zero();
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // cross matrix accumulation
  cross_ = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  dv_db_g_ = Eigen::Matrix3d::Zero();
  dp_db_g_ = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  // Clear vectors for preintegrated values at rts residual time.
  Delta_q_vec_.clear();
  C_doubleintegral_vec_.clear();
  C_integral_vec_.clear();
  acc_integral_vec_.clear();
  acc_doubleintegral_vec_.clear();
  dalpha_db_g_vec_.clear();
  dp_db_g_vec_.clear();
  dv_db_g_vec_.clear();
  square_root_information_vec_.clear();

  bool has_started = false;
  bool last_iteration = false;
  int n_integrated = 0;

  size_t n_to_integrate_at_t_rts = static_cast<size_t>(num_residuals()/3);

  for (size_t i = imu_measurements_.size()-1; i != 0u; --i)
  {
    Eigen::Vector3d omega_S_0 = imu_measurements_[i].angular_velocity_;
    Eigen::Vector3d acc_S_0 = imu_measurements_[i].linear_acceleration_;
    Eigen::Vector3d omega_S_1 = imu_measurements_[i-1].angular_velocity_;
    Eigen::Vector3d acc_S_1 = imu_measurements_[i-1].linear_acceleration_;
    double nexttime = imu_measurements_[i - 1].timestamp_;

    // time delta
    double dt = nexttime - time;

    // Sanity check
    if (dt <= 0.0)
    {
      continue;
    }

    if (!has_started)
    {
      has_started = true;
      const double r = dt / (nexttime - imu_measurements_[i].timestamp_);
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imu_parameters_.sigma_g_c;
    double sigma_a_c = imu_parameters_.sigma_a_c;

    if (std::abs(omega_S_0[0]) > imu_parameters_.g_max
        || std::abs(omega_S_0[1]) > imu_parameters_.g_max
        || std::abs(omega_S_0[2]) > imu_parameters_.g_max)
    {
      sigma_g_c *= 100;
      LOG(WARNING)<< "gyr saturation";
    }

    if (std::abs(acc_S_0[0]) > imu_parameters_.a_max
        || std::abs(acc_S_0[1]) > imu_parameters_.a_max
        || std::abs(acc_S_0[2]) > imu_parameters_.a_max)
    {
      sigma_a_c *= 100;
      LOG(WARNING)<< "acc saturation";
    }

    // Get propagation terms at residual time
    if (gp_measurements_[n_to_integrate_at_t_rts - 1].timestamp_ <= nexttime)
    {
      n_to_integrate_at_t_rts--;

      if (n_to_integrate_at_t_rts == 0)
      {
        last_iteration = true;
      }

      // Interpolate
      double interval = nexttime - imu_measurements_[i].timestamp_;
      double dt_till_t_rts = gp_measurements_[n_to_integrate_at_t_rts].timestamp_ - time;
      const double r = dt_till_t_rts / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();

      // ensure integrity
      if (std::abs(omega_S_1[0]) > imu_parameters_.g_max
          || std::abs(omega_S_1[1]) > imu_parameters_.g_max
          || std::abs(omega_S_1[2]) > imu_parameters_.g_max)
      {
        sigma_g_c *= 100;
        LOG(WARNING)<< "gyr saturation";
      }

      if (std::abs(acc_S_1[0]) > imu_parameters_.a_max
          || std::abs(acc_S_1[1]) > imu_parameters_.a_max
          || std::abs(acc_S_1[2]) > imu_parameters_.a_max)
      {
        sigma_a_c *= 100;
        LOG(WARNING)<< "acc saturation";
      }

      // actual propagation
      // orientation:
      Eigen::Quaterniond dq;
      const Eigen::Vector3d omega_S_true =
          (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt_till_t_rts;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt_till_t_rts;
      dq.w() = cos_theta_half;
      Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
      // rotation matrix integral:
      const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
      const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
      const Eigen::Vector3d acc_S_true =
          (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases.segment<3>(6));
      const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt_till_t_rts;
      const Eigen::Vector3d acc_integral_1 =
          acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt_till_t_rts;
      // rotation matrix double integral:
      const Eigen::Matrix3d C_doubleintegral_at_t_rts = C_doubleintegral_ +
              C_integral_ * dt_till_t_rts + 0.25 * (C + C_1) * dt_till_t_rts * dt_till_t_rts;
      const Eigen::Vector3d acc_doubleintegral_at_t_rts = acc_doubleintegral_ +
              acc_integral_ * dt_till_t_rts
              + 0.25 * (C + C_1) * acc_S_true * dt_till_t_rts * dt_till_t_rts;

      // Jacobian parts
      const Eigen::Matrix3d dalpha_db_g_at_t_rts = dalpha_db_g_
              + C_1 * expmapDerivativeSO3(omega_S_true * dt_till_t_rts) * dt_till_t_rts;
      const Eigen::Matrix3d cross_1 =
          dq.inverse().toRotationMatrix() * cross_
          + expmapDerivativeSO3(omega_S_true * dt_till_t_rts) * dt_till_t_rts;
      const Eigen::Matrix3d acc_S_x = skewSymmetric(acc_S_true);
      Eigen::Matrix3d dv_db_g_1 =
          dv_db_g_ + 0.5 * dt_till_t_rts * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      const Eigen::Matrix3d dp_db_g_at_t_rts = dp_db_g_ +
              dt_till_t_rts * dv_db_g_
              + 0.25 * dt_till_t_rts * dt_till_t_rts
              * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);

      // covariance propagation
      Eigen::Matrix<double, 15, 15> F_delta =
          Eigen::Matrix<double, 15, 15>::Identity();
      // transform
      F_delta.block<3, 3>(0, 3) =
          -skewSymmetric(acc_integral_ * dt_till_t_rts
                         + 0.25 * (C + C_1) * acc_S_true * dt_till_t_rts * dt_till_t_rts);
      F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt_till_t_rts;
      F_delta.block<3, 3>(0, 9) =
          dt_till_t_rts * dv_db_g_
          + 0.25 * dt_till_t_rts * dt_till_t_rts
              * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(0, 12) = -C_integral_ * dt_till_t_rts
              + 0.25 * (C + C_1) * dt_till_t_rts * dt_till_t_rts;
      F_delta.block<3, 3>(3, 9) = -dt_till_t_rts * C_1;
      F_delta.block<3, 3>(6, 3) =
          -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt_till_t_rts);
      F_delta.block<3, 3>(6, 9) =
          0.5 * dt_till_t_rts * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt_till_t_rts;

      Eigen::Matrix<double, 15, 15> P_delta_at_t_rts =
              Eigen::Matrix<double, 15, 15>::Identity();;
      P_delta_at_t_rts = F_delta * P_delta_ * F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      //F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt_till_t_rts * sigma_g_c * sigma_g_c;
      P_delta_at_t_rts(3, 3) += sigma2_dalpha;
      P_delta_at_t_rts(4, 4) += sigma2_dalpha;
      P_delta_at_t_rts(5, 5) += sigma2_dalpha;
      const double sigma2_v = dt_till_t_rts * sigma_a_c * sigma_a_c;
      P_delta_at_t_rts(6, 6) += sigma2_v;
      P_delta_at_t_rts(7, 7) += sigma2_v;
      P_delta_at_t_rts(8, 8) += sigma2_v;
      const double sigma2_p = 0.5 * dt_till_t_rts * dt_till_t_rts * sigma2_v;
      P_delta_at_t_rts(0, 0) += sigma2_p;
      P_delta_at_t_rts(1, 1) += sigma2_p;
      P_delta_at_t_rts(2, 2) += sigma2_p;
      const double sigma2_b_g =
          dt_till_t_rts * imu_parameters_.sigma_gw_c * imu_parameters_.sigma_gw_c;
      P_delta_at_t_rts(9, 9) += sigma2_b_g;
      P_delta_at_t_rts(10, 10) += sigma2_b_g;
      P_delta_at_t_rts(11, 11) += sigma2_b_g;
      const double sigma2_b_a =
          dt_till_t_rts * imu_parameters_.sigma_aw_c* imu_parameters_.sigma_aw_c;
      P_delta_at_t_rts(12, 12) += sigma2_b_a;
      P_delta_at_t_rts(13, 13) += sigma2_b_a;
      P_delta_at_t_rts(14, 14) += sigma2_b_a;

      // get the weighting:
      // enforce symmetric
      P_delta_at_t_rts = 0.5 * P_delta_at_t_rts + 0.5 * P_delta_at_t_rts.transpose().eval();

      // calculate inverse
      Eigen::Matrix<double, 15, 15> information;
      information = P_delta_at_t_rts.inverse();
      information = 0.5 * information + 0.5 * information.transpose().eval();

      // square root
      Eigen::Matrix<double, 15, 15> square_root_information;
      Eigen::LLT<Eigen::Matrix<double, 15, 15>> lltOfInformation(information);
      square_root_information = lltOfInformation.matrixL().transpose();

      // store quantities
      Delta_q_vec_.push_back(Delta_q_1);
      C_doubleintegral_vec_.push_back(C_doubleintegral_at_t_rts);
      C_integral_vec_.push_back(C_integral_1);
      acc_integral_vec_.push_back(acc_integral_1);
      acc_doubleintegral_vec_.push_back(acc_doubleintegral_at_t_rts);
      dalpha_db_g_vec_.push_back(dalpha_db_g_at_t_rts);
      dp_db_g_vec_.push_back(dp_db_g_at_t_rts);
      dv_db_g_vec_.push_back(dv_db_g_1);
      P_delta_vec_.push_back(P_delta_at_t_rts);
      square_root_information_vec_.push_back(square_root_information);
    }

    if (last_iteration)
    {
      break;
    }
    else
    {
      // ensure integrity
      if (std::abs(omega_S_1[0]) > imu_parameters_.g_max
          || std::abs(omega_S_1[1]) > imu_parameters_.g_max
          || std::abs(omega_S_1[2]) > imu_parameters_.g_max)
      {
        sigma_g_c *= 100;
        LOG(WARNING)<< "gyr saturation";
      }

      if (std::abs(acc_S_1[0]) > imu_parameters_.a_max
          || std::abs(acc_S_1[1]) > imu_parameters_.a_max
          || std::abs(acc_S_1[2]) > imu_parameters_.a_max)
      {
        sigma_a_c *= 100;
        LOG(WARNING)<< "acc saturation";
      }

      // actual propagation
      // orientation:
      Eigen::Quaterniond dq;
      const Eigen::Vector3d omega_S_true =
              (0.5 * (omega_S_0 + omega_S_1) - speed_and_biases.segment<3>(3));
      const double theta_half = omega_S_true.norm() * 0.5 * dt;
      const double sinc_theta_half = sinc(theta_half);
      const double cos_theta_half = cos(theta_half);
      dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
      dq.w() = cos_theta_half;
      Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
      // rotation matrix integral:
      const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();
      const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
      const Eigen::Vector3d acc_S_true =
              (0.5 * (acc_S_0 + acc_S_1) - speed_and_biases.segment<3>(6));
      const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;
      const Eigen::Vector3d acc_integral_1 =
              acc_integral_ + 0.5 * (C + C_1) * acc_S_true * dt;
      // rotation matrix double integral:
      C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
      acc_doubleintegral_ +=
              acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

      // Jacobian parts
      dalpha_db_g_ += C_1 * expmapDerivativeSO3(omega_S_true * dt) * dt;
      const Eigen::Matrix3d cross_1 =
              dq.inverse().toRotationMatrix() * cross_
              + expmapDerivativeSO3(omega_S_true * dt) * dt;
      const Eigen::Matrix3d acc_S_x = skewSymmetric(acc_S_true);
      Eigen::Matrix3d dv_db_g_1 =
              dv_db_g_ + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      dp_db_g_ += dt * dv_db_g_
              + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);

      // covariance propagation
      Eigen::Matrix<double, 15, 15> F_delta =
          Eigen::Matrix<double, 15, 15>::Identity();
      // transform
      F_delta.block<3, 3>(0, 3) =
          -skewSymmetric(acc_integral_ * dt
                         + 0.25 * (C + C_1) * acc_S_true * dt * dt);
      F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
      F_delta.block<3, 3>(0, 9) =
          dt * dv_db_g_
          + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(0, 12) = -C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;
      F_delta.block<3, 3>(3, 9) = -dt * C_1;
      F_delta.block<3, 3>(6, 3) =
          -skewSymmetric(0.5 * (C + C_1) * acc_S_true * dt);
      F_delta.block<3, 3>(6, 9) =
          0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
      F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
      P_delta_ = F_delta * P_delta_ * F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be
      // ignored, since the noise is isotropic.
      //F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta_(3, 3) += sigma2_dalpha;
      P_delta_(4, 4) += sigma2_dalpha;
      P_delta_(5, 5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * sigma_a_c;
      P_delta_(6, 6) += sigma2_v;
      P_delta_(7, 7) += sigma2_v;
      P_delta_(8, 8) += sigma2_v;
      const double sigma2_p = 0.5 * dt * dt * sigma2_v;
      P_delta_(0, 0) += sigma2_p;
      P_delta_(1, 1) += sigma2_p;
      P_delta_(2, 2) += sigma2_p;
      const double sigma2_b_g =
          dt * imu_parameters_.sigma_gw_c * imu_parameters_.sigma_gw_c;
      P_delta_(9, 9) += sigma2_b_g;
      P_delta_(10, 10) += sigma2_b_g;
      P_delta_(11, 11) += sigma2_b_g;
      const double sigma2_b_a =
          dt * imu_parameters_.sigma_aw_c* imu_parameters_.sigma_aw_c;
      P_delta_(12, 12) += sigma2_b_a;
      P_delta_(13, 13) += sigma2_b_a;
      P_delta_(14, 14) += sigma2_b_a;

      // memory shift
      Delta_q_ = Delta_q_1;
      C_integral_ = C_integral_1;
      acc_integral_ = acc_integral_1;
      cross_ = cross_1;
      dv_db_g_ = dv_db_g_1;
      time = nexttime;

      ++n_integrated;
    }

  }

  // store the reference (linearisation) point
  speed_and_biases_ref_ = speed_and_biases;

  return n_integrated;
}

// This evaluates the error term and additionally computes the Jacobians.
bool GpError::Evaluate(double const* const * parameters,
                        double* residuals,
                        double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes the Jacobians.
bool GpError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const
{
  // get poses
  const Transformation T_WS_0(
        Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3],
      parameters[0][4], parameters[0][5]));

  // get speed and bias
  SpeedAndBias speed_and_biases_0;
  for (size_t i = 0; i < 9; ++i)
  {
    speed_and_biases_0[i] = parameters[1][i];
  }

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.getRotationMatrix();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double d_t = t1_ - t0_;
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
    Delta_b = speed_and_biases_0.tail<6>()
        - speed_and_biases_ref_.tail<6>();
  }

  // Same check as in imu_error.
  redo_ = redo_ || (Delta_b.head<3>().norm() * d_t > 0.0001);
  if (redo_)
  {
    redoPreintegration(T_WS_0, speed_and_biases_0);

    redoCounter_++;
    Delta_b.setZero();
    redo_ = false;
    /*if (redoCounter_ > 1) {
      std::cout << "pre-integration no. " << redoCounter_ << std::endl;
    }*/
  }

  // actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegration_mutex_);
    // this is a bit stupid, but shared read-locks only come in C++14
    const Eigen::Vector3d g_W = Eigen::Vector3d(0, 0, imu_parameters_.g);

    const size_t n_residuals_terms = static_cast<size_t>(num_residuals());

    const size_t n_residuals = static_cast<size_t>(n_residuals_terms/3);

    // Map residuals for Ceres.
    Eigen::Matrix<double, Eigen::Dynamic, 1> weighted_error =
            Eigen::MatrixXd::Zero(n_residuals_terms, 1);

    // Store minimal jacobian wrt position and orientation for all residuals.
    std::vector< Eigen::Matrix<double, 3, 6> > J0_minimal_vec;
    //std::vector< Eigen::Matrix<double, 15, 6> > J0_minimal_vec;
    // Store jacobian wrt speed and biases
    std::vector< Eigen::Matrix<double, 3, 9> > J1_vec;
    //std::vector< Eigen::Matrix<double, 15, 9> > J1_vec;

    // Rts measurement.
    // p: prism nodal point, S: sensor(=body of VIO sensor) frame,
    // W: total station ref. frame
    const Eigen::Vector3d S_p_PS = gp_parameters_.B_t_PB;

    for (size_t k = 0; k < n_residuals; ++k)
    {
      size_t ind = n_residuals - 1 - k;
      const Eigen::Vector3d W_p_WP1 = gp_measurements_[ind].position_;
      const double delta_t = gp_measurements_[ind].timestamp_ - t0_;

      // assign Jacobian w.r.t. x0
      // Here it is similar to imu error but C_WS1 is estimated propagating imu meas.
      // from T_WS_0. Same for speed_1.
      const Eigen::Quaterniond Dq =
          deltaQ(-dalpha_db_g_vec_[k]*Delta_b.head<3>())*Delta_q_vec_[k];
      const Eigen::Quaterniond q_WS1_est = T_WS_0.getEigenQuaternion() * Dq;
      /*const Eigen::Matrix3d C_WS1_est = T_WS_0.getRotationMatrix() * Dq.toRotationMatrix();
      const Eigen::Quaterniond q_WS1_est(C_WS1_est);*/
      const Eigen::Vector3d W_p_WS1 = W_p_WP1 + q_WS1_est.toRotationMatrix() * S_p_PS;

      const Eigen::Vector3d delta_p_est_W =
          T_WS_0.getPosition() - W_p_WS1
          + speed_and_biases_0.head<3>()* delta_t - 0.5 * g_W* delta_t * delta_t;

      /*const Eigen::Vector3d speed_1 = speed_and_biases_0.head<3>() - g_W * delta_t +
              C_WS_0 *
              (acc_integral_vec_[k] +
               dv_db_g_vec_[k] * Delta_b.segment<3>(0)
               - C_integral_vec_[k] * Delta_b.segment<3>(3));

      const Eigen::Vector3d delta_v_est_W = speed_and_biases_0.head<3>()
          - speed_1 - g_W * delta_t;

      const Eigen::Vector3d delta_v_est_W = -C_WS_0 *
              (acc_integral_vec_[k]
               + dv_db_g_vec_[k] * Delta_b.segment<3>(0)
               - C_integral_vec_[k] * Delta_b.segment<3>(3));*/

      Eigen::Matrix<double,3,15> F0 =
          Eigen::Matrix<double,3,15>::Identity(); // holds for d/db_g, d/db_a
      F0.block<3,3>(0,0) = C_S0_W;
      F0.block<3,3>(0,3) = C_S0_W * skewSymmetric(delta_p_est_W);
      F0.block<3,3>(0,6) = C_S0_W * Eigen::Matrix3d::Identity()* delta_t;
      F0.block<3,3>(0,9) = dp_db_g_vec_[k];
      F0.block<3,3>(0,12) = -C_doubleintegral_vec_[k];

      // the overall error vector
      // Since we do not estimate speed and biases at rts residual,
      // the corresponding errors are 0 and so not computed.
      Eigen::Matrix<double, 3, 1> error;
      error =
          C_S0_W * delta_p_est_W + acc_doubleintegral_vec_[k]
              + F0.block<3,6>(0,9)*Delta_b;

      // error weighting
      // Tot covariance
      Eigen::Matrix3d covariance_tot = C_S0_W * gp_covariance_ * C_WS_0
                                   + P_delta_vec_[k].block<3,3>(0,0);
      // enforce symmetric
      covariance_tot  = 0.5 * covariance_tot + 0.5 * covariance_tot.transpose().eval();

      // Tot information
      Eigen::Matrix3d information_tot = covariance_tot.inverse();
      // enforce symmetric
      information_tot = 0.5 * information_tot + 0.5 * information_tot.transpose().eval();

      // Total square root information
      Eigen::LLT<Eigen::Matrix3d> lltOfInformation(information_tot);
      Eigen::Matrix3d square_root_information_tot;
      square_root_information_tot = lltOfInformation.matrixL().transpose();

      weighted_error.segment<3>(3*k) = square_root_information_tot * error;

      // Compute and store minimal jacobian
      Eigen::Matrix<double, 3, 6> J0pos_minimal_this_residual =
              square_root_information_tot * F0.block<3, 6>(0, 0);
      J0_minimal_vec.push_back(J0pos_minimal_this_residual);

      Eigen::Matrix<double, 3, 9> J1_this_residual_full =
                    square_root_information_tot * F0.block<3, 9>(0, 6);
      J1_vec.push_back(J1_this_residual_full);
    }

    // Map residuals
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(residuals, n_residuals_terms, 1)
            = weighted_error;

    // get the Jacobians
    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        Eigen::MatrixXd J0 = Eigen::MatrixXd::Zero(n_residuals_terms, 7);

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        for (size_t k = 0; k < n_residuals; ++k)
        {
          J0.block<3,7>(3*k,0) = J0_minimal_vec[k] * J_lift;
        }

        // Map jacobian
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                (jacobians[0], n_residuals_terms, 7) = J0;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[0] != nullptr)
          {
            Eigen::MatrixXd J0_minimal = Eigen::MatrixXd::Zero(n_residuals_terms, 6);

            for (size_t k = 0; k < n_residuals; ++k)
            {
              J0_minimal.block<3,6>(3*k,0) = J0_minimal_vec[k];//.block<3,6>(0,0);
            }

            // Map jacobian
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                    (jacobians_minimal[0], n_residuals_terms, 6) = J0_minimal;
          }
        }
      }

      if (jacobians[1] != nullptr)
      {
        Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(n_residuals_terms, 9);

        for (size_t k = 0; k < n_residuals; ++k)
        {
          J1.block<3,9>(3*k,0) = J1_vec[k];//.block<3,9>(0,0);
        }

        // Map jacobian
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                (jacobians[1], n_residuals_terms, 9) = J1;

        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                    (jacobians_minimal[1], n_residuals_terms, 9) = J1;
          }
        }
      }
    }
  }

  return true;
}

}  // namespace ceres_backend
}  // namespace svo
