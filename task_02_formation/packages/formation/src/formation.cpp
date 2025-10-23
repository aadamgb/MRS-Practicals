#include <student_headers/formation.h>

namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

bool bpm_dfs(int row, std::vector<std::vector<int>>& adj, std::vector<bool>& visited, std::vector<int>& match) {
  for (int col : adj[row]) {
    if (!visited[col]) {
      visited[col] = true;
      
      if (match[col] == -1 || bpm_dfs(match[col], adj, visited, match)) {
        match[col] = row;
        return true;
      }

    }
  }
  return false;
}


std::vector<Eigen::Vector2d> find_independent_zeros(std::vector<std::vector<double>>& cost) {
  int n = cost.size();

  // Build adjacency list of zero positions
  std::vector<std::vector<int>> adj(n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (fabs(cost[i][j]) < 1e-9) adj[i].push_back(j);
    }
  }

  std::vector<int> match_col(n, -1);
  for (int row = 0; row < n; row++) {
    std::vector<bool> visited(n, false);
    bpm_dfs(row, adj, visited, match_col);
  }

  std::vector<Eigen::Vector2d> independent_zeros;
  for (int j = 0; j < n; ++j) {
    if (match_col[j] != -1) {
      independent_zeros.push_back(Eigen::Vector2d(match_col[j], j));
    }
  }

  return independent_zeros;
}

std::vector<int> Hungarian(std::vector<std::vector<double>>& cost) {
  int n = cost.size();
  // | ---- Step 1: Row and column reduction ---- |
    // Step 1: Row reduction
    for (int i = 0; i < n; i++) {
        double min_val = *std::min_element(cost[i].begin(), cost[i].end());
        for (int j = 0; j < n; j++) {
            cost[i][j] -= min_val;
        }
    }
    
    // Step 1: Column reduction
    for (int j = 0; j < n; j++) {
        double min_val = cost[0][j];
        for (int i = 1; i < n; i++) {
            if (cost[i][j] < min_val) min_val = cost[i][j];
        }
        for (int i = 0; i < n; i++) {
            cost[i][j] -= min_val;
        }
    }


  // | --- Step 2: Find the solution ------------ |
  while(true) {
    auto zeros  = find_independent_zeros(cost);
  if ((int)zeros.size() == n) {
    // Final assignment
    std::vector<int> assignment(n, -1);
    for (auto& z : zeros) {
      assignment[z[0]] = z[1];
    }
    return assignment;
  }

    std::vector<bool> covered_rows(n, false);
    std::vector<bool> covered_cols(n, false);

    // Step 3: Cover all rows without assigned zeros
    for (int i = 0; i < n; i++){
      bool assigned = false;
      for (auto& z : zeros) {
        if ((int)z[0] == i) assigned = true;
      }
      if (!assigned) covered_rows[i] = true;
    }

    
    // Step 4: Iteratively mark columns and 
    bool changed;
    do {
      changed = false;
      for (int i = 0; i < n; i++) {
        if (covered_rows[i]) {
          for ( int j = 0; j < n; j++) {
            if (fabs(cost[i][j]) < 1e-9 && !covered_cols[j]) {
              covered_cols[j] = true;
              for (auto& z : zeros) {
                if ((int)z[1] == j && !covered_rows[(int)z[0]]) {
                  covered_rows[(int)z[0]] = true;
                  changed = true;
                }
              }
            }
          }
        }
      }
    } while (changed);

    // Step 5: Find the mimimum uncoverd value
    double min_uncovered = std::numeric_limits<double>::max();
    for (int i = 0; i < n; i++) {
      if (!covered_rows[i]) {
        for (int j = 0; j < n; j++) {
          if (!covered_cols[j] && cost[i][j] < min_uncovered) {
            min_uncovered = cost[i][j];
          }
        }
      }
    }

    // Step 6: Adjust cost matrix
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        if (!covered_rows[i] && !covered_cols[j])
          cost[i][j] -= min_uncovered;
        else if (covered_rows[i] && covered_cols[j])
          cost[i][j] += min_uncovered;
      }
    }
  }
}

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {
}

//}

/* getPathsReshapeFormation() //{ */

/**
 * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
 * This method is supposed to be filled in by the student.
 *
 * @param initial_states A vector of 3D initial positions for each UAV.
 * @param final_states A vector of 3D final positions of each UAV.
 *
 * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
 * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
 * point for a UAV, can be, e.g.:
 *   I -> F
 *   I -> B -> F
 *   I -> B -> C -> F
 * The following paths are considered invalid:
 *   I
 *   F
 *   D -> D
 *   I -> D
 *   F -> I
 */
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                              const std::vector<Eigen::Vector3d> &final_states) {

  // use the visualizeCube() method
  // * very useful for showing the obstacle set for the Astar
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers_.visualizeCube(Position_t{0, 0, 0}, Color_t{0.0, 0.0, 1.0, 0.1}, 1.0);

  // how many UAVs do we have
  int n_uavs = initial_states.size();

  // | ------------- calculate the cost matrix (length from start to goals) ------------ |
  std::vector<std::vector<double>> cost(n_uavs, std::vector<double>(n_uavs));

  for(int i = 0; i < n_uavs; i++) {
    for(int j = 0; j < n_uavs; j++) {
      cost[i][j] = (final_states[j] - initial_states[i]).norm();
    }
  }

  std::vector<int> assignment = Hungarian(cost);

  // initialize the vector of paths
  std::vector<std::vector<Eigen::Vector3d>> paths(n_uavs);

  // for each UAV
  for (int i = 0; i < n_uavs; i++) {

    // prepare the path
    std::vector<Eigen::Vector3d> path;

    // path made of two waypoints: I -> F
    path.push_back(initial_states[i]);
    path.push_back(final_states[assignment[i]]);

    paths.push_back(path);
  }

  return paths;
}



//}

/* multilateration() //{ */

/**
 * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
 *
 * @param uav_states Vector of 3D positions of each UAV.
 * @param distances Vector of the measured distances from each UAV to the source of signal.
 *
 * @return the estimated 3D position of the source of radiation.
 */
Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions, const Eigen::VectorXd &distances) {

  // THIS IS THE MOST BASIC OPTIMIZATION FOR THE POSITION OF THE ROBOT
  // The method can be improved significantly by:
  // * increasing the number of iterations
  // * trying multiple different initial conditions (xk)
  // * not optimizing for the full 3D position of the robot, we know that the robot rides on the ground, z = 0
  // * using better optimization method (LM)

  const int N = int(positions.size());

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N, 3);
  Eigen::MatrixXd g = Eigen::VectorXd::Zero(N);

  // the solution... initialized as (0, 0, 0)^T, is it a good initialization?
  Eigen::Vector3d s = Eigen::Vector3d(0, 0, 0);

  const int max_iterations = 30;

  for (int n_iterations = 0; n_iterations < max_iterations; n_iterations++) {

    for (int j = 0; j < N; j++) {

      J.row(j) = (s - positions[j]) / (s - positions[j]).norm();
    }

    // distance from xk to the sphere with radius distances[i] and center positions[i]
    for (int i = 0; i < N; i++) {
      g(i) = (s - positions[i]).norm() - distances[i];
    }

    // do the Gauss-Newton iteration
    s = s - (J.transpose() * J).inverse() * J.transpose() * g;
  }

  return s;
}

//}

/* update() //{ */

/**
 * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz, and,
 * therefore, it should not be blocking.
 *
 * @param formation_state The current state of the formation. The state contains:
 * - absolute position of the virtual leader
 * - positions of the follower UAVs relative the virtual leader
 * - flag stating whether the formation is moving or whether it is stationary
 * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
 * @param time_stamp Current time in seconds.
 * @param action_handlers This structure provides users with functions to control the formation:
 *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
 *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
 * Moreover, the action_handlers structure provides additional methods for data visualization.
 */
void Formation::update(const FormationState_t &formation_state, const Ranging_t &ranging, [[maybe_unused]] const double &time_stamp,
                       ActionHandlers_t &action_handlers) {

  // how many UAVs are there in the formation?
  const int n_uavs = int(formation_state.followers.size());

  // | ------------- calculate the target's position ------------ |

  // calculate the abolsute positions of the formation members
  std::vector<Eigen::Vector3d> abs_positions;

  for (int i = 0; i < n_uavs; i++) {
    abs_positions.push_back(formation_state.followers[i] + formation_state.virtual_leader);
  }

  Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);

  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);

  // | ------------------- Put your code here ------------------- |

  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }

  // this is an example of a "state machine"
  switch (user_defined_variable_) {

    // in the fist state, reorganize the formation into a column
    case 0: {

      std::vector<Eigen::Vector3d> formation_line;
      formation_line.push_back(Eigen::Vector3d(-3.0, 0.0, 3.0));
      formation_line.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));
      formation_line.push_back(Eigen::Vector3d(3.0, 0.0, 3.0));

      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      }

      user_defined_variable_++;

      break;
    }

    case 1: {

      // tell the virtual leader to move to the center of the arena
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(0, 0, 3));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      }

      user_defined_variable_++;

      break;
    }

    default: {

      // tell the virtual leader to move to the next "cylinder"
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(10.0 * (user_defined_variable_ - 2), 0, 3));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      }

      user_defined_variable_++;

      break;
    }
  }
}

//}

}  // namespace task_02_formation
