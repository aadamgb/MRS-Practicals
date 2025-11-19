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

  // Call custom functions, e.g., useful for dynamic weighting
  [[maybe_unused]] double x = multiply(5.0, 10.0);

  // Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();

  // Am I nearby a beacon?
  Distribution beacon_distribution;
  if (state.nearby_beacon) {
    beacon_distribution = state.beacon_distribution;
  }

  // Iterate over the states of the visible neighbors
  for (const auto &n_state : state.neighbors_states) {

    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    // distance to the neighbour
    [[maybe_unused]] double n_dist = n_pos_rel.norm();

    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }
  }

  // Example: scale the action by user parameter
  action = user_params.param1 * target;

  // Print the output action
  printVector3d(action, "Action: ");

  // Visualize the arrow in RViz
  action_handlers.visualizeArrow("action", action, Color_t{0.0, 0.0, 0.0, 1.0});

  // | -------------------- EXAMPLE CODE END -------------------- |

  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
