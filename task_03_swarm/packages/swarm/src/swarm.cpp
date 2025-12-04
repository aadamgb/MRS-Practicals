#include <student_headers/swarm.h>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

#define PI 3.141516

/* init() //{ */

/**
 * @brief The swarm controller (single unit) initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param visibility radius of the agent
 */
void Swarm::init(const double visibility_radius) {

  _visibility_radius_ = visibility_radius; 

}

//}

// | ------ Compulsory functions to be filled by students ----- |

/* updateAction() //{ */

/**
 * @brief This method calculates a next-iteration action of one UAV given relative information of its neighbors and obstacles; and the direction towards the
 *        moving target. This method is supposed to be filled in by the student.
 *
 * @param perception Current perceptual information of this UAV. Defined in perception.h. It contains:
 *  - current time
 *  - target vector: 3D vector towards the moving robot in the UAV body frame
 *  - neighbors defined by:
 *    - their position in the UAV body frame
 *    - the variables shared through the communication network
 *  - obstacles consisting of:
 *    - 3D vector from the body frame to the closest obstacle in the environment
 *    - 4 gates (pairs of 2 gate edges) in the UAV body frame
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization and data sharing among the UAVs:
 *  - shareVariables(int, int, double) will share the three basic-type variables among the UAVs
 *  - visualizeArrow() will publish the given arrow in the UAV body frame within the visualization
 *  - visualizeArrowFrom() will publish the given arrow at position given in the UAV body frame within the visualization
 *  - visualizeCube() will publish a cube in the UAV body frame within the visualization
 *
 * @return Next-iteration action for this UAV given as a 3D vector. Zero vector is expected if no action should be performed. Beware that i) the vector z-axis
 * component will be set to 0, ii) the vector vector magnitude will be clamped into <0, v_max> limits, and iii) the vector will be passed to a velocity
 * controller of the UAV.
 *
 *       Example 1: v_max=0.75 -> vector (1, 0, 1) will be saturated to 0.75*(1, 0, 0).
 */
Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.

  // | ------------------- EXAMPLE CODE START ------------------- |

  // Setup output control signal
  Eigen::Vector3d vec_action = Eigen::Vector3d::Zero();

  // Access the perception struct
  double          current_time    = perception.time;

  Eigen::Vector3d vec_navigation  = Eigen::Vector3d::Zero();
  Eigen::Vector3d vec_cohesion    = Eigen::Vector3d::Zero();
  Eigen::Vector3d vec_separation  = Eigen::Vector3d::Zero();

  // Access custom parameters
  double param1 = user_params.param1;
  double param2 = user_params.param2;
  double param3 = user_params.param3;

  // Variables initialization
  bool compute_action = false;

  Direction_t selected_gate_dir;

  // STATE MACHINE BEGINNING
  switch (_state_) {

      /* case INIT_STATE //{ */

    case INIT_STATE: {

      _state_          = AGREEING_ON_DIRECTION;
      idling_time_init = current_time;
      _navigation_direction_ = NONE;

      break;
    }

      //}

      /* case AGREEING_ON_DIRECTION() //{ */

    case AGREEING_ON_DIRECTION: {

      if (_navigation_direction_ == NONE) {
        _navigation_direction_ = targetToDirection(perception.target_vector);
        action_handlers.shareVariables(_state_, _navigation_direction_, 10.0);
      }

      // Compute majority
      std::vector<int> directions                                = {directionToInt(_navigation_direction_)};
      directions.push_back(perception.neighbors[0].shared_variables.int2);
      directions.push_back(perception.neighbors[1].shared_variables.int2);

      auto             counts                                    = countIntegers(directions);
      const auto &[majority_idx, majority_freq] = getMajority(counts);

      bool direction_agreed = true;

      if (direction_agreed) {
        compute_action = true;
        selected_gate_dir = intToDirection(majority_idx);
      }

      break;
    }

      //}
  }

  // STATE MACHINE END

  if (compute_action) {
    double closest_n_dist = std::numeric_limits<double>::max();
    bool collision_risk = false;
    bool get_closer = false; 

    // | --------------- Separate from other agents --------------- |
    for (const auto &n : perception.neighbors) {
      Eigen::Vector3d n_pos  = n.position;
      double          n_dist = n_pos.norm();

      vec_cohesion += n_pos; // coheison force

      bool   weight_defined;      double weight;
      std::tie(weight_defined, weight) = weightingFunction(n_dist, _visibility_radius_, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS);

      if (weight_defined) {
        vec_separation += -n_pos * weight;
      } else {
        vec_separation += -100000.0 * n_pos; // Saturate separation force
      }

      closest_n_dist = std::min(closest_n_dist, n_dist);
    }

    if(closest_n_dist < DESIRED_DISTANCE_UAVS) {
      collision_risk = true;
    } else {
      collision_risk = false;
    }
    
    auto mutual_distances = computeMutualDistances(perception.neighbors);
    for (size_t i = 0; i < mutual_distances.size(); i++) {
      if(mutual_distances[i] < MAX_MUTUAL_DISTANCE) {
        get_closer = false;
      } else {
        get_closer = true;
      }
    }

    // | ----------------- Separate from obstacles ---------------- |
    double closest_gp_dist = perception.obstacles.closest.norm();
    unsigned int gate_in_direction_idx = selectGateInDirection(selected_gate_dir, perception.obstacles);
    auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];

    double gate_dist = ((gate_in_direction.first + gate_in_direction.second) / 2.0).norm();
    action_handlers.shareVariables(_state_, _navigation_direction_, gate_dist);

    // compute minimum gate_distance between all uavs
    double min_val = std::min({gate_dist,
                   perception.neighbors[0].shared_variables.dbl,
                   perception.neighbors[1].shared_variables.dbl});

    vec_navigation = gate_in_direction.first + gate_in_direction.second; 

    vec_navigation = vec_navigation.normalized() / (gate_dist * gate_dist);
    vec_cohesion = (vec_cohesion / perception.neighbors.size());
    vec_separation += - perception.obstacles.closest.normalized() / (closest_gp_dist * closest_gp_dist);

    // | ---------------------- weight forces --------------------- |
    // Turn of some forces in case of:
    if (collision_risk) {
      vec_navigation = Eigen::Vector3d::Zero();
      vec_cohesion = Eigen::Vector3d::Zero();
      std::cout << "❌[COLLISON RISK]❌  Setting nav and cohesion forces to zero " << std::endl;
    } 
    
    if(get_closer){
      vec_navigation = Eigen::Vector3d::Zero();
      std::cout << "😭[Feeling lonley]😭  Setting nav force to zero " << std::endl;
    }
    

    // | ------------------- sum the subvectors ------------------- |
    vec_action = vec_navigation * param1 + vec_separation * param2 + vec_cohesion * param3 +  perception.target_vector * 0.15; // small target comonent added

    if (current_time - idling_time_init >= VOTING_TIME && min_val > 4.0) {
      _state_ = INIT_STATE;
      idling_time_init = current_time;
      action_handlers.shareVariables(_state_, _navigation_direction_, gate_dist);
      std::cout << "🗳️ VOTING!! Resetting to INIT_STATE" << std::endl;
    }


    // | ------------------------ visualize ----------------------- |
    action_handlers.visualizeArrow("separation", vec_separation, Color_t{1.0, 0.0, 0.0, 0.5});        // red
    action_handlers.visualizeArrow("cohesion", vec_cohesion, Color_t{0.0, 1.0, 0.0, 0.5});            // green
    action_handlers.visualizeArrow("G_p", gate_in_direction.first, Color_t{0.0, 0.0, 1.0, 0.2});      // blue
    action_handlers.visualizeArrow("G_n", gate_in_direction.second, Color_t{0.0, 0.0, 1.0, 0.2});     // blue
    action_handlers.visualizeArrow("closest_gp", -perception.obstacles.closest.normalized() / (closest_gp_dist * closest_gp_dist), Color_t{1.0, 0.0, 1.0, 0.5}); // purple

  } // end compute action 

  action_handlers.visualizeArrow("target", perception.target_vector * 2.0, Color_t{1.0, 1.0, 1.0, 0.5});
  action_handlers.visualizeArrow("action", vec_action, Color_t{0.0, 0.0, 0.0, 1.0});

  // | -------------------- EXAMPLE CODE END -------------------- |

  return vec_action;
}

//}

/* weightingFunction() //{ */

/**
 * @brief Non-linear weighting of forces.
 *
 * The function is to be non-increasing, non-negative, and grows to infinity as the distance is approaching the lower bound (the safety distance). Below the
 * lower bound (including), the function is to be undefined. Over the visibility range, the function shall return 0.
 *
 * @param distance to an agent/obstacle
 * @param visibility visibility range of the UAVs
 * @param safety distance: min distance to other UAVs or obstacles
 * @param desired distance: desired distance to other UAVs or obstacles (does not need to be used)
 *
 * @return
 *   bool:   True if function is defined for the given distance, False otherwise
 *   double: Weight for an agent/obstacle at given distance, if function is defined for the given distance.
 */
std::tuple<bool, double> Swarm::weightingFunction(const double distance, const double visibility, const double safety_distance,
                                                  [[maybe_unused]] const double desired_distance) {

  double weight;
  if (safety_distance < distance && distance <= visibility) {
    weight = 1.0 / (distance - safety_distance);
    return {true, weight};
  } else if (visibility < distance) {
    return {true, 0.0};
  } else {
    return {false, visibility - distance};
  }

}

//}

// | -- Helper methods to be filled in by students if needed -- |

/* targetToDirection() //{ */

Direction_t Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  // TODO: fill if want to use
  // std::cout << "[ERROR] targetToDirection() not implemented. Returning UP by default." << std::endl;

  double x = target_vector.x();
  double y = target_vector.y();

  double angle  = atan2(y,x);
  // std::cout << "[ANGLE] The angle is: " << angle * 180.0 / PI << " deg" << std::endl;

  if (-PI/4.0 < angle && angle <= PI/4.0) {
    // - 45 deg to + 45 deg -> go RIGHT
    return RIGHT;
  } else if (PI/4.0 < angle  && angle <= 3*PI/4.0) {
    // + 45 deg to + 135 deg -> go UP
    return UP;
  } else if (-3*PI/4.0  < angle && angle <= -PI/4.0){
    // - 45 deg to -135 deg -> go DOWN
    return DOWN;
  } else {
    // + 135 deg + 135 deg -> go LEFT
    return LEFT;
  }

}

//}

// /* robotsInIdenticalStates() //{ */

// bool Swarm::robotsInIdenticalStates(const Perception_t &perception) {

//   // TODO: fill if want to use
//   std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning false if there are any neighbors, true otherwise." << std::endl;

//   for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
//     return false;
//   }

//   return true;
// }

// //}

// /* anyRobotInState() //{ */

// bool Swarm::anyRobotInState(const Perception_t &perception, const State_t &state) {

//   // TODO: fill if want to use
//   std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning true if there are any neighbors, false otherwise." << std::endl;

//   for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
//     return true;
//   }

//   return false;
// }

//}




// | ------------ Helper methods for data handling ------------ |

/* selectGateInDirection() //{ */

/**
 * @brief Finds the index of the gate in the given direction.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[dir_idx])
 */
unsigned int Swarm::selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles) {

  switch (direction) {

    case UP: {
      return 1;

      break;
    }

    case DOWN: {
      return 3;

      break;
    }

    case LEFT: {
      return 2;

      break;
    }

    case RIGHT: {
      return 0;

      break;
    }

    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;

      break;
    }
  }

  return 0;
}

//}

/* selectGateClosest() //{ */

/**
 * @brief Finds the index of the gate closest to the agent.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[min_idx])
 */
unsigned int Swarm::selectGateClosest(const Obstacles_t &obstacles) {

  unsigned int min_idx  = 0;
  double       min_dist = obstacles.gates[0].first.norm();

  for (unsigned int i = 0; i < obstacles.gates.size(); i++) {

    const auto   G      = obstacles.gates[i];
    const double G_dist = (G.first.norm() < G.second.norm()) ? G.first.norm() : G.second.norm();

    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}

//}

/* computeMutualDistances() //{ */

/**
 * @brief Computes the vector of mutual distances between agents
 *
 * @return vector of all mutual distances (unordered) between all agents (incl. me)
 */
std::vector<double> Swarm::computeMutualDistances(const std::vector<Neighbor_t> &neighbors) {

  // All known positions (incl. mine)
  std::vector<Eigen::Vector3d> positions = {Eigen::Vector3d::Zero()};
  for (const auto &n : neighbors) {
    positions.push_back(n.position);
  }

  // Compute all mutual distances
  std::vector<double> distances;
  for (unsigned int i = 0; i < positions.size(); i++) {
    for (unsigned int j = i + 1; j < positions.size(); j++) {
      distances.push_back((positions[j] - positions[i]).norm());
    }
  }

  return distances;
}

//}

/* integersAreUnique() //{ */

/**
 * @brief Check if integers in a vector are unique
 *
 * @return true if all the integers are unique
 */
bool Swarm::integersAreUnique(const std::vector<int> &integers) {

  const auto count_map = countIntegers(integers);

  return count_map.size() == integers.size();
}

//}

/* countIntegers() //{ */

/* Computes frequency of integers in the given array
 *
 * @return map of counts for each key in the initial list
 * */
std::map<int, int> Swarm::countIntegers(const std::vector<int> &integers) {

  std::map<int, int> count_map;

  for (const int i : integers) {
    if (count_map.find(i) == count_map.end()) {
      count_map[i] = 0;
    }
    count_map[i]++;
  }

  return count_map;
}

//}

/* getMajority() //{ */

/* Return the key and value of the first maximal element in the given idx->count map.
 *
 * @return key, count
 * */
std::tuple<int, int> Swarm::getMajority(const std::map<int, int> &integer_counts) {

  if (integer_counts.empty()) {
    return {-1, -1};
  }

  int max_idx = 0;
  int max_val = 0;

  for (auto it = integer_counts.begin(); it != integer_counts.end(); ++it) {
    if (it->second > max_val) {
      max_idx = it->first;
      max_val = it->second;
    }
  }

  return {max_idx, max_val};
}

std::tuple<int, int> Swarm::getMajority(const std::vector<int> &integers) {
  return getMajority(countIntegers(integers));
}

//}

// | --------- Helper methods for data-type conversion -------- |

/* stateToInt() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
int Swarm::stateToInt(const State_t &state) {
  return static_cast<int>(state);
}

//}

/* intToState() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
State_t Swarm::intToState(const int value) {
  return static_cast<State_t>(value);
}

//}

// | --------------- Helper methods for printing -------------- |

/* stateToString() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
std::string Swarm::stateToString(const State_t &state) {

  // TODO: fill with your states if you want to use this method
  switch (state) {

    case INIT_STATE: {
      return "INIT_STATE";

      break;
    }

    case AGREEING_ON_DIRECTION: {
      return "AGREEING_ON_DIRECTION";

      break;
    }

    default: {
      break;
    }
  }

  return "UNKNOWN";
}

//}

}  // namespace task_03_swarm
