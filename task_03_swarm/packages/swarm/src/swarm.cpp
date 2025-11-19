#include <student_headers/swarm.h>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

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
  double          current_time   = perception.time;
  Eigen::Vector3d vec_navigation = perception.target_vector;
  Eigen::Vector3d vec_separation = Eigen::Vector3d::Zero();

  // Access custom parameters
  double param1 = user_params.param1;
  double param2 = user_params.param2;

  // Variables initialization
  bool compute_action = false;

  // STATE MACHINE BEGINNING
  switch (_state_) {

      /* case INIT_STATE //{ */

    case INIT_STATE: {

      std::cout << "Current state: " << stateToString(INIT_STATE) << std::endl;

      std::cout << "Changing to state: " << stateToString(AGREEING_ON_DIRECTION) << std::endl;
      _state_          = AGREEING_ON_DIRECTION;
      idling_time_init = current_time;

      // You may share three variables to all other agents (beware, the network is asynchronous and UDP-based: no message is assured to reach all the others)
      // CPP enum values are represented by integers by default in ascending order. You may hence send your own state to others like this:
      action_handlers.shareVariables(INIT_STATE, 0, 0.0);

      // We have prepared two basic enums for you: State_t defined in student_headers/swarm.h and task_03_common/Direction_t in direction.h.
      // You may share both and add a double value, e.g.:
      _navigation_direction_ = NONE;
      action_handlers.shareVariables(INIT_STATE, _navigation_direction_, 3.1415);

      // The values of others can be accessed like this:
      // SharedVariables_t n_0_shared_vars = perception.neighbors[0].shared_variables;
      // int a = n_0_shared_vars.int1;
      // int b = n_0_shared_vars.int2;
      // double c = n_0_shared_vars.dbl;

      break;
    }

      //}

      /* case AGREEING_ON_DIRECTION() //{ */

    case AGREEING_ON_DIRECTION: {

      std::cout << "Current state: " << stateToString(AGREEING_ON_DIRECTION) << std::endl;

      if (_navigation_direction_ == NONE) {
        _navigation_direction_ = targetToDirection(perception.target_vector);
      }

      // Compute majority
      std::vector<int> directions                                = {directionToInt(_navigation_direction_)};
      auto             counts                                    = countIntegers(directions);
      [[maybe_unused]] const auto &[majority_idx, majority_freq] = getMajority(counts);

      bool direction_agreed = true;

      if (direction_agreed) {
        std::cout << "Selected direction: " << directionToString(_navigation_direction_) << std::endl;
      }

      break;
    }

      //}
  }

  // STATE MACHINE END

  if (compute_action) {

    // | --------------- Separate from other agents --------------- |

    for (const auto &n : perception.neighbors) {
      Eigen::Vector3d n_pos  = n.position;
      double          n_dist = n_pos.norm();

      // You may want to use the weighting function you should have prepared first
      bool   weight_defined;
      double weight;
      std::tie(weight_defined, weight) = weightingFunction(n_dist, _visibility_radius_, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS);

      if (weight_defined) {
        // probably use the weight
      } else {
        // possibly use some backup weight
      }
    }

    // | ----------------- Separate from obstacles ---------------- |

    auto gates = perception.obstacles.gates;

    // You may access the gates relative to your body frame
    Eigen::Vector3d G1_p = gates[0].first;
    Eigen::Vector3d G1_n = gates[0].second;

    // Or you may iterate over them
    for (const auto &G : gates) {
      const auto G_p = G.first;
    }

    // Or you may want to find:
    //  the closest gate:
    unsigned int closest_gate_idx = selectGateClosest(perception.obstacles);
    auto         closest_gate     = perception.obstacles.gates[closest_gate_idx];

    //  the gate for the direction:
    unsigned int gate_in_direction_idx = selectGateInDirection(UP, perception.obstacles);
    auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];

    // | ---------------------- weight forces --------------------- |
    vec_navigation *= param1;
    vec_separation *= param2;

    // | ------------------- sum the subvectors ------------------- |
    vec_action = vec_navigation + vec_separation;
    printVector3d(vec_action, "Action:");

    // | ------------------------ visualize ----------------------- |
    action_handlers.visualizeArrow("separation", vec_separation, Color_t{1.0, 0.0, 0.0, 0.5});
    action_handlers.visualizeArrow("navigation", vec_navigation, Color_t{0.0, 0.0, 1.0, 0.5});
  }

  action_handlers.visualizeArrow("target", perception.target_vector, Color_t{1.0, 1.0, 1.0, 0.5});
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

  // TODO: Filling this function is compulsory!

  return {false, visibility - distance};
}

//}

// | -- Helper methods to be filled in by students if needed -- |

/* targetToDirection() //{ */

Direction_t Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  // TODO: fill if want to use
  std::cout << "[ERROR] targetToDirection() not implemented. Returning UP by default." << std::endl;

  double x = target_vector.x();
  double y = target_vector.y();

  return UP;
}

//}

/* robotsInIdenticalStates() //{ */

bool Swarm::robotsInIdenticalStates(const Perception_t &perception) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning false if there are any neighbors, true otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return false;
  }

  return true;
}

//}

/* anyRobotInState() //{ */

bool Swarm::anyRobotInState(const Perception_t &perception, const State_t &state) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning true if there are any neighbors, false otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return true;
  }

  return false;
}

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
