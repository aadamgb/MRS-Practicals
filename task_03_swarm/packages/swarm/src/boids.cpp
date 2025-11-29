#include <task_03_boids/boids.h>

namespace task_03_boids
{

// | ---------- HERE YOU MAY WRITE YOUR OWN FUNCTIONS --------- |

// Example function, can be deleted
double multiply(const double a, const double b) {
  return a * b;
}

// | ------------- FILL COMPULSORY FUNCTIONS BELOW ------------ |

/* updateAgentState() //{ */

/**
 * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
 *        This method is supposed to be filled in by the student.
 *
 * @param AgentState_t Current state of the agent as defined in agent_state.h.
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization
 *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
 *
 * @return
 *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
 * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's current
 * velocity and the vector does not exceed a maximal change.
 *
 *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
 * 0.2*(cos(d), sin(d), 0).
 *
 *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, -0.05, 1) will
 * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
 *
 *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
 * state.distribution.dim().
 */
std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.

  // | ------------------- EXAMPLE CODE START ------------------- |
  // Setup the output action
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = state.target; 

  // Boids forces 
  Eigen::Vector3d f_a  = Eigen::Vector3d::Zero();  // Alignment
  Eigen::Vector3d f_c  = Eigen::Vector3d::Zero();  // Cohesion
  Eigen::Vector3d f_s  = Eigen::Vector3d::Zero();  // Separation
  int n_neighbours =  state.neighbors_states.size();

  // Hard safety collision limits
  const double MIN_DIST_LIMIT = 0.3;
  const double SAFETY_BUFFER = 0.05; 
  bool collision_risk = false;
  
  // Call custom functions, e.g., useful for dynamic weighting
  [[maybe_unused]] double x = multiply(5.0, 10.0);

  // Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();

  // | ------------------ MAIN LOOP ------------------------- |
  // | ---------- neighbours iteration loop ----------------- |
  for (const auto &n_state : state.neighbors_states) {
    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    double n_dist = n_pos_rel.norm();
    if (n_dist < 0.01) n_dist = 0.01; // safety measure; prevent dividing by zero

    if (n_dist < (MIN_DIST_LIMIT + SAFETY_BUFFER)) {
      collision_risk = true;
      // separation force proportional to 1/x^2
      f_s += -n_pos_rel.normalized() / (n_dist * n_dist); 
    }

    if(!collision_risk){
      // No collision risk
      f_a +=   n_vel_global;
      f_c +=   n_pos_rel;
      f_s += - n_pos_rel.normalized() / n_dist;
    }

    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }

    // Adding neighbours distributions to my distribution
    for (int i = 0; i < 4; i++){
      my_distribution.add(i, n_distribution.get(i));
    }

  }

  // | ----------- BOIDS ACTIONS CALCULATIONS ------------------- |
  if (collision_risk){
    // Only take into account the separation force saturate it to maximal vel. change
    action = f_s * 500.0;

  } else if (n_neighbours > 0) {
    // Nominal case
    f_a.normalize();
    f_c = (f_c / n_neighbours).normalized();
    action = (f_a * user_params.param1 + f_c * user_params.param2 + f_s * user_params.param3) + target * user_params.param9;

  } else {
    // No neighbours, just go for the target
    action = target * user_params.param9;
  }

  // Visualize the arrow in RViz
  action_handlers.visualizeArrow("action", action*4, Color_t{1.0, 0.0, 0.0, 1.0});

  // | -------------------- SETTING DISTRIBUTIONS -------------------- |
  if(state.nearby_beacon) {
    my_distribution = state.beacon_distribution;
  } else {
    my_distribution.normalize();
  }

  // | ---------------------- DEBUG PRINTS ------------------------ |
  // printVector3d(action, "Action: ");
  // printVector3d(f_a, "Velocity alignment: ");
  // std::cout << "My distribution: " << my_distribution << std::endl;

  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
