#include <student_headers/controller.h>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief the prediction step of the LKF
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param input current control input: input = [tilt_xz, tilt_yz, acceleration_z]^T, tilt_xz = desired tilt in the world's XZ [rad], tilt_yz = desired
 * tilt in the world's YZ plane [rad], acceleration_z = desired acceleration along world's z-axis [m/s^2]
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfPredict(const Vector9d &x, const Matrix9x9d &x_cov, const Vector3d &input, const double &dt) {

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  new_x = A_ * x + B_ * input;
  new_x_cov = A_ * x_cov * A_.transpose() + Q_;

  /* Print the following values into a log file.
   * the file will be place in "simulation/student_log.txt"
   * use this for ploting in custom scipts, e.g., using Matlab or Python. */
   
  std::stringstream string_to_be_logged;
  string_to_be_logged << std::fixed << dt << ", " << x[0] << ", " << x[1] << ", " << x[2];
  action_handlers_.logLine(string_to_be_logged);

  return {new_x, new_x_cov};
}

/**
 * @brief LKF filter correction step
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param measurement measurement vector: measurement = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfCorrect(const Vector9d &x, const Matrix9x9d &x_cov, const Vector6d &measurement, const double &dt) {

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  // Kalman gain 
  K_ = x_cov * H_.transpose() *
      (H_ * x_cov * H_.transpose() + R_).inverse();

  // Updated state prediction 
  new_x = x + K_ * (measurement - H_ * x);

  // Updated covariance (Joseph form for numerical stability)
  new_x_cov = (Matrix9x9d::Identity() - K_ * H_) * x_cov;
  return {new_x, new_x_cov};
}

}  // namespace task_01_controller
