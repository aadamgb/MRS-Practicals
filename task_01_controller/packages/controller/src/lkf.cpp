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

  Matrix9x9d A = Matrix9x9d::Zero();
  Matrix9x9d B = Matrix9x9d::Zero();
  Matrix9x9d Q = Matrix9x9d::Zero();

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix
  
  for (int i = 0; i < 3; ++i) {
    // p_{k+1} = p_k + v_k*dt + 0.5*a_k*dt^2
    A(i, i)       = 1.0;                      // p_k
    A(i, i + 3)   = dt;                       // v_k * dt
    A(i, i + 6)   = 0.5 * dt * dt;            // 0.5*a_k*dt^2

    // v_{k+1} = v_k + a_k*dt
    A(i + 3, i + 3) = 1.0;                    // v_k
    A(i + 3, i + 6) = dt;                     // a_k*dt

    // a_{k+1} = a_k
    A(i + 6, i + 6) = 1.0;                    // a_k
  }

  B(6, 0) = _g_;                              // tilt_xz (alpha_d)
  B(7, 1) = _g_;                              // tilt_yz (beta_d)
  B(8, 2) = 1.0;                              // acceleration_z
  
  Q.block<3,3>(0,0) = 1e-6 * Matrix3d::Identity(); // pos noise 
  Q.block<3,3>(3,3) = 1e-5 * Matrix3d::Identity(); // vel noise 
  Q.block<3,3>(6,6) = 1e-3 * Matrix3d::Identity(); // acc noise

  new_x = A * x + B * input;
  new_x_cov = A * x_cov * A.transpose() + Q;

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  //
  // std::stringstream string_to_be_logged;
  // string_to_be_logged << std::fixed << dt << ", " << x[0] << ", " << x[1] << ", " << x[2];
  // action_handlers_.logLine(string_to_be_logged);

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

  // PUT YOUR CODE HERE
  // new_x = ...
  // new_x_cov = ...

  return {new_x, new_x_cov};
}

}  // namespace task_01_controller
