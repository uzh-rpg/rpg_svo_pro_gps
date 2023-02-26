/**
 * @file global_position_error.hpp
 * @brief Header file for the GpError class. Reference: imu_error.hpp
 * @author Giovanni Cioffi cioffi at <cioffi at ifi dot uzh dot ch>.
 */

#pragma once

#include <mutex>

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since
// c++11.
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#pragma diagnostic pop

#include <svo/common/types.h>
#include <svo/common/imu_calibration.h>
#include <svo/common/global_positions_settings.h>
#include <svo/vio_common/logging.hpp>

#include "svo/ceres_backend/estimator_types.hpp"
#include "svo/ceres_backend/error_interface.hpp"
#include "svo/ceres_backend/imu_error.hpp"


namespace svo {
namespace ceres_backend {

/// \brief Implements robotic total station factor.
class GpError :
    public ceres::CostFunction,
    public ErrorInterface
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::deque<Transformation,
    Eigen::aligned_allocator<Transformation> > Transformations;

  typedef std::deque<SpeedAndBias,
    Eigen::aligned_allocator<SpeedAndBias> > SpeedsAndBiases;

  typedef std::deque<Eigen::Matrix<double,15, 15>,
    Eigen::aligned_allocator<Eigen::Matrix<double,15, 15>>> Covariances;

  typedef std::deque<Eigen::Matrix<double,15, 15>,
    Eigen::aligned_allocator<Eigen::Matrix<double,15, 15>>> Jacobians;

  /// \brief Default constructor -- assumes information recomputation.
  GpError() = default;

  /// \brief Trivial destructor.
  virtual ~GpError() = default;

  /// \brief Construct with measurements and parameters.
  /// \@param[in] gp_measurements The global position measurements including timestamp
  /// \@param[in] gp_parameters The parameters to be used.
  /// \@param[in] imu_measurements The IMU measurements including timestamp
  /// \@param[in] imu_params The parameters to be used.
  /// \@param[in] t_0 Start time.
  /// \@param[in] t_1 End time.
  GpError(const GpMeasurements &gp_measurements,
           const GpParameters& gp_parameters,
           const ImuMeasurements &imu_measurements,
           const ImuParameters& imu_parameters,
           const double &t_0,
           const double &t_1);

  /**
   * @brief Propagates pose, speeds and biases with given IMU measurements.
   * @remark This function is basically only used to test computation in redoPreintegration
   * @param[in] imu_measurements The IMU measurements including timestamp
   * @param[in] imu_params The parameters to be used.
   * @param[in] T_WS0 Start pose[in].
   * @param[out] T_WS pose at time rts measurement.
   * @param[inout] speed_and_biases Start speed and biases.
   * @param[in] t_start Start time.
   * @param[in] t_end End time.
   * @param[out] covariance Covariance for GIVEN start states.
   * @param[out] jacobian Jacobian w.r.t. start states.
   * @return Number of integration steps.
   */
  static int propagation(const ImuMeasurements &imu_measurements,
                         const ImuParameters& imu_parameters,
                         const GpMeasurements &rts_measurements,
                         Transformation& T_WS0,
                         Transformations &T_WS,
                         SpeedAndBias& speed_and_biases0,
                         SpeedsAndBiases &speed_and_biases,
                         const double &t_start,
                         const bool compute_covariance,
                         const bool compute_jacobian,
                         Covariances &covariances,
                         Jacobians &jacobians);

  /**
   * @brief Propagates pose, speeds and biases with given IMU measurements.
   * @warning This is not actually const, since the re-propagation must somehow
   *          be stored...
   * @param[in] T_WS Start pose.
   * @param[in] speed_and_biases Start speed and biases.
   * @return Number of integration steps.
   */
  int redoPreintegration(const Transformation& T_WS,
                         const SpeedAndBias& speed_and_biases) const;

  /**
   * @brief Finds for each rts measurement the index of the first imu measurements next in time.
   * @param[in] T_WS Start pose.
   * @param[in] speed_and_biases Start speed and biases..
   */
  void setGpImuIndexCorrespondences();

  // setters
  void setRedo(const bool redo = true) const
  {
    redo_ = redo;
  }

  /// \brief (Re)set the parameters.
  /// \@param[in] GpParameters The parameters to be used.
  void setGpParameters(const GpParameters& gpParameters)
  {
    gp_parameters_ = gpParameters;
  }

  /// \brief (Re)set the measurements
  void setGpMeasurements(const GpMeasurements& gp_measurements)
  {
    gp_measurements_ = gp_measurements;
  }

  /// \brief (Re)set the parameters.
  /// \@param[in] imuParameters The parameters to be used.
  void setImuParameters(const ImuParameters& imuParameters)
  {
    imu_parameters_ = imuParameters;
  }

  /// \brief (Re)set the measurements
  void setImuMeasurements(const ImuMeasurements& imu_measurements)
  {
    imu_measurements_ = imu_measurements;
  }

  /// \brief (Re)set the start time.
  /// \@param[in] t_0 Start time.
  void setT0(const double& t_0) { t0_ = t_0; }

  /// \brief (Re)set the next frame timestamp.
  /// \@param[in] t_nextframe End time.
  void setT1(const double& t_1) { t1_ = t_1; }

  /// \brief Set covariance.
  void setGpCovariance()
  {
    gp_covariance_ = gp_parameters_.cov;
  }

  /// \brief Set information and squared information matrix.
  void setId(const int id){ id_ = id; }

  // getters

  /// \brief Get the Gp Parameters.
  /// \return the gp parameters.
  const GpParameters& gpParameters() const
  {
    return gp_parameters_;
  }

  /// \brief Get the Rts measurements.
  const GpMeasurements gpMeasurements() const
  {
    return gp_measurements_;
  }

  /// \brief Get the IMU Parameters.
  /// \return the IMU parameters.
  const ImuParameters& imuParameters() const
  {
    return imu_parameters_;
  }

  /// \brief Get the IMU measurements.
  const ImuMeasurements imuMeasurements() const
  {
    return imu_measurements_;
  }

  /// \brief Get the start time.
  double getT0() const { return t0_; }

  /// \brief Get the nextframe time.
  double getT_nextframe() const { return t1_; }

  /// \brief Get the last gp measurement time.
  double getT_last_gp() const { return gp_measurements_.front().timestamp_; }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobians_minimal Pointer to the minimal Jacobians
   *        (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobians_minimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const
  {
    return static_cast<size_t>(num_residuals());
  }

  /// \brief Number of parameter blocks.
  virtual size_t parameterBlocks() const
  {
    return parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameter_block_idx Index of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return ceres::CostFunction::parameter_block_sizes().at(parameter_block_idx);
  }

  /// @brief Return parameter block type as string
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kGpError;
  }

 protected:
  // parameters
  ImuParameters imu_parameters_; ///< The IMU parameters.
  GpParameters gp_parameters_; ///< The Gp parameters.

  // measurements
  ImuMeasurements imu_measurements_;
  GpMeasurements gp_measurements_;

  // times
  double t0_; ///< The start time (i.e. time of the first set of states).
  double t1_; ///< The time of the next frame (used to check if repropagate).

  // For each rts meas., index of first imu meas. next in time.
  // Position-> rts measurement, value-> index of first imu measurement next in time.
  std::vector<int> index_first_imu_meas_next_in_time_;

  // preintegration stuff. the mutable is a TERRIBLE HACK, but what can I do.
  mutable std::mutex preintegration_mutex_;
  ///< Protect access of intermediate results.

  // increments (initialise with identity)
  mutable Eigen::Quaterniond Delta_q_ = Eigen::Quaterniond(1,0,0,0);
  mutable Eigen::Matrix3d C_integral_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d C_doubleintegral_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Vector3d acc_integral_ = Eigen::Vector3d::Zero();
  mutable Eigen::Vector3d acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  mutable Eigen::Matrix3d cross_ = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  mutable Eigen::Matrix3d dalpha_db_g_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d dv_db_g_ = Eigen::Matrix3d::Zero();
  mutable Eigen::Matrix3d dp_db_g_ = Eigen::Matrix3d::Zero();

  /// \brief The Jacobian of the increment (w/o biases).
  mutable Eigen::Matrix<double,15,15> P_delta_ =
          Eigen::Matrix<double,15,15>::Zero();

  // increments vectors
  mutable std::vector<Eigen::Quaterniond> Delta_q_vec_;
  mutable std::vector<Eigen::Matrix3d> C_doubleintegral_vec_;
  mutable std::vector<Eigen::Matrix3d> C_integral_vec_;
  mutable std::vector<Eigen::Vector3d> acc_integral_vec_;
  mutable std::vector<Eigen::Vector3d> acc_doubleintegral_vec_;
  mutable std::vector<Eigen::Matrix3d> dalpha_db_g_vec_;
  mutable std::vector<Eigen::Matrix3d> dp_db_g_vec_;
  mutable std::vector<Eigen::Matrix3d> dv_db_g_vec_;
  mutable std::vector<Eigen::Matrix<double, 15, 15>> P_delta_vec_;
  mutable std::vector<Eigen::Matrix<double, 15, 15>> square_root_information_vec_;

  /// \brief Reference biases that are updated when called redoPreintegration.
  mutable SpeedAndBias speed_and_biases_ref_ = SpeedAndBias::Zero();

  mutable bool redo_ = true;
  ///< Keeps track of whether or not this redoPreintegration() needs to be done.
  mutable int redoCounter_ = 0;
  ///< Counts the number of preintegrations for statistics.

  // Gp covariance, information matrix and its square root
  mutable Eigen::Matrix3d gp_covariance_;

  /// ID
  size_t id_ = 0;

};

}  // namespace ceres_backend
}  // namespace svo
