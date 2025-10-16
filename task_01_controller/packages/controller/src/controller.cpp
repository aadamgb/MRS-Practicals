#include <task_01_controller/utils.h>

#include <student_headers/controller.h>

#include <iostream>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param mass UAV mass [kg]
 * @param user_params user-controllable parameters
 * @param g gravitational acceleration [m/s^2]
 * @param action_handlers methods for the user
 */
void Controller::init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) {

  // copy the mass and the gravity acceleration
  this->_mass_ = mass;
  this->_g_    = g;

  // the action handlers will allow you to plot data
  this->action_handlers_ = action_handlers;

  // INITIALIZE YOUR CONTROLLER HERE
  _error_.setZero();
  _prev_error_.setZero();
  _int_error_.setZero();

  this->first_iteration_ = true;

  // INITIALIZE YOUR KALMAN FILTER HERE
  A_ = Matrix9x9d::Identity();  B_ = Matrix9x3d::Zero(); 
  H_ = Matrix6x9d::Zero();      K_ = Matrix9x6d::Zero();
  Q_ = Matrix9x9d::Identity();  R_ = Matrix6x6d::Identity();

  A_.diagonal().tail<3>()           = Vector3d(0.95, 0.95, 0.99);
  A_.diagonal(3)                    = 0.01 * Vector6d::Ones();
  A_.diagonal(6)                    = 0.5 * 0.01 * 0.01 * Vector3d::Ones();

  B_.block<3, 3>(6, 0).diagonal()   = _g_ * Vector3d(0.05, 0.05, 0.01);
  
  H_.block<3,3>(0,0).diagonal()     = Vector3d::Ones();                         // px, py, pz
  H_.block<3,3>(3,6).diagonal()     = Vector3d::Ones();                         // ax, ay, az

  Q_.block<3,3>(0,0)                = 0.001 * Matrix3d::Identity();             // pos process noise 
  Q_.block<3,3>(3,3)                = 0.001 * Matrix3d::Identity();              // vel process noise 
  Q_.block<3,3>(6,6)                = 1.0 * Matrix3d::Identity();                // acc process noise

  R_.block<3,3>(0,0)                = 1.0 * Matrix3d::Identity();                // pos measurement noise
  R_.block<3,3>(3,3)                = 1.0 * Matrix3d::Identity();                // acc measurement noise

  // SET THE STATE AND THE COVARIANCE MATRICES AS GLOBAL VARIABLES
  x_.setZero();                                                  
  x_cov_.setIdentity();
}

/**
 * @brief This method is called to reset the internal state of the controller, e.g., just before
 * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
 */
void Controller::reset() {

  // IT WOULD BE GOOD TO RESET THE PID'S INTEGRALS
  _error_.setZero();
  _prev_error_.setZero();
  _int_error_.setZero();

  // IT WOULD BE NICE TO RESET THE KALMAN'S STATE AND COVARIANCE
  x_.setZero();
  x_cov_.setIdentity();

  // ALSO, THE NEXT iteration calculateControlSignal() IS GOING TO BE "THE 1ST ITERATION"
  this->first_iteration_ = true;
}

/**
 * @brief the main routine, is called to obtain the control signals
 *
 * @param uav_state the measured UAV state, contains position and acceleration
 * @param user_params user-controllable parameters
 * @param control_reference the desired state of the UAV, position, velocity, acceleration, heading
 * @param dt the time difference in seconds between now and the last time calculateControlSignal() got called
 *
 * @return the desired control signal: the total thrust force and the desired orientation
 */
std::pair<double, Matrix3d> Controller::calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                               const ControlReference_t control_reference, const double dt) {

  // Publish the following values as "ROS topics" such that they can be plotted
  // * plotting can be achived using, e.g., the tool called PlotJuggler
  // * try the "plot.sh" script, which will run PlotJuggler
  action_handlers_.plotValue("pos_x", uav_state.position[0]);
  action_handlers_.plotValue("pos_x_kalman", x_[0]);
  
  action_handlers_.plotValue("pos_y", uav_state.position[1]);
  action_handlers_.plotValue("pos_y_kalman", x_[1]);

  action_handlers_.plotValue("pos_z", uav_state.position[2]);
  action_handlers_.plotValue("pos_z_kalman", x_[2]);

  action_handlers_.plotValue("vel_x_kalman", x_[3]);
  action_handlers_.plotValue("vel_y_kalman", x_[4]);
  action_handlers_.plotValue("vel_z_kalman", x_[5]);

  action_handlers_.plotValue("acc_x", uav_state.acceleration[0]);
  action_handlers_.plotValue("acc_x_kalman", x_[6]);
  
  action_handlers_.plotValue("acc_z", uav_state.acceleration[2]);
  action_handlers_.plotValue("acc_z_kalman", x_[8]);


  // publish the following pose as "ROS topic", such that it can be plotted by Rviz
  action_handlers_.visualizePose("uav_pose_offset", uav_state.position[0], uav_state.position[1], uav_state.position[2] + 1.0, uav_state.heading);

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  
  std::stringstream string_to_be_logged;
  string_to_be_logged << std::fixed << dt << ", " << uav_state.position[0] << ", " << uav_state.position[1] << ", " << uav_state.position[2];
  action_handlers_.logLine(string_to_be_logged);

  //  | --- Initialize State on first iteration --- |
  if (first_iteration_) {
    x_.head<3>() = uav_state.position;                    // Initial position
    x_.segment<3>(6) = uav_state.acceleration;            // Initial acceleration
    first_iteration_ = false;
  }

  // | ---------- Calculate the output control signals ---------- |
  _error_ = control_reference.position - x_.head<3>();

  _int_error_ += _error_ * dt; 

  double control_x = user_params.param1 * _error_[0] + 
                     user_params.param2 * _int_error_[0] +
                     user_params.param3 * (_error_[0] - _prev_error_[0]) / dt;
            

  double control_y = user_params.param1 * _error_[1] + 
                     user_params.param2 * _int_error_[1] +
                     user_params.param3 * (_error_[1] - _prev_error_[1]) / dt;

  double control_z = user_params.param4 * _error_[2] + 
                     user_params.param5 * _int_error_[2] +
                     user_params.param6 * (_error_[2] - _prev_error_[2]) / dt;
  
  _prev_error_ = _error_;

  // | -------------- Calculate desired body tilts -------------- |
  double des_tilt_x = control_x + 
  std::asin(std::clamp(control_reference.acceleration[0] / (control_reference.acceleration[2] + _g_), -1.0, 1.0));
  
  double des_tilt_y = control_y + 
  std::asin(std::clamp(control_reference.acceleration[1] / (control_reference.acceleration[2] + _g_), -1.0, 1.0));
  
  double des_accel_z = control_z + control_reference.acceleration[2];                                                 


  // | ------ Tune process and measurment noise covariance  ------ |
  Q_.block<3,3>(0,0) = user_params.param7 * Matrix3d::Identity(); // pos noise 
  Q_.block<3,3>(3,3) = user_params.param8 * Matrix3d::Identity(); // vel noise 
  Q_.block<3,3>(6,6) = user_params.param9 * Matrix3d::Identity(); // acc noise

  R_.block<3,3>(0,0) = user_params.param10 * Matrix3d::Identity(); // pos noise
  R_.block<3,3>(3,3) = user_params.param11 * Matrix3d::Identity(); // acc noise
                                                        
  // | ------------ Create input and measurment vectors ----------- |
  Vector3d input;   Vector6d measurement;
  input << des_tilt_x, des_tilt_y, des_accel_z;
  measurement << uav_state.position, uav_state.acceleration;

  // | --------------- Return the control signals --------------- |
  double   body_thrust;
  Matrix3d desired_orientation;
  std::tie(body_thrust, desired_orientation) = augmentInputs(des_tilt_x, des_tilt_y, (des_accel_z + _g_) * _mass_, control_reference.heading);

  // | ------------ Future state estimation with LKF ------------ |
  std::tie(x_, x_cov_) = lkfPredict(x_, x_cov_, input, dt);
  std::tie(x_, x_cov_) = lkfCorrect(x_, x_cov_, measurement, dt);

  return {body_thrust, desired_orientation};
};

}  // namespace task_01_controller
