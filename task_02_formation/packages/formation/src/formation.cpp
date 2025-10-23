#include <student_headers/formation.h>

namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

std::vector<int> hungarianSolve(const std::vector<std::vector<double>> &cost) {
  int n = cost.size();
  int m = cost[0].size();
  int dim = std::max(n, m);
  std::vector<std::vector<double>> a(dim, std::vector<double>(dim, 0.0));

  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j)
      a[i][j] = cost[i][j];

  std::vector<double> u(dim + 1), v(dim + 1);
  std::vector<int> p(dim + 1), way(dim + 1);

  for (int i = 1; i <= dim; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(dim + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(dim + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (int j = 1; j <= dim; ++j) {
        if (used[j]) continue;
        double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= dim; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= dim; ++j)
    if (p[j] <= n && j <= m)
      assignment[p[j] - 1] = j - 1;
  return assignment;
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
      cost[i][j] = (initial_states[i] - final_states[j]).norm();
    }
  }

  // | ------------- Assign optimal goals to starts with Hungarian Algorithm ------------ |
  std::vector<int> assignment = hungarianSolve(cost);

  // initialize the vector of paths
  std::vector<std::vector<Eigen::Vector3d>> paths;

 // | --------------- Initialize the Astar object -------------- |
  const double resolution = 0.6;
  astar::Astar astar(resolution);
  std::set<astar::Cell> obstacles;
  double separation = 1.8;  // minimum distance between paths


  // for each UAV
  for (int i = 0; i < n_uavs; i++) {

  // prepare the path container
  std::vector<Eigen::Vector3d> path;

  // initialize A* start and goal
  astar::Position start(initial_states[i][0], initial_states[i][1], initial_states[i][2]);
  astar::Position goal(final_states[assignment[i]][0], final_states[assignment[i]][1], final_states[assignment[i]][2]);

  // Create temporary obstacles set for this planning iteration
  std::set<astar::Cell> temp_obstacles = obstacles;

  // | -------- Add starts of other UAVs as temporary obstacles -------- |
  std::set<astar::Cell> other_starts_obstacles;
  for (int j = 0; j < n_uavs; j++) {
    if (j != i) {  // Skip the current UAV's own start
      astar::Cell other_start = astar.toGrid(initial_states[j][0], initial_states[j][1], initial_states[j][2]);
      
      // Expand obstacle region around other UAV's start position
      int expand_cells = static_cast<int>(std::ceil(separation / resolution));
      
      for (int dx = -expand_cells; dx <= expand_cells; dx++) {
        for (int dy = -expand_cells; dy <= expand_cells; dy++) {
          for (int dz = -expand_cells; dz <= expand_cells; dz++) {
            
            // Convert back to physical coordinates
            double dist = std::sqrt(std::pow(dx * resolution, 2) +
                                    std::pow(dy * resolution, 2) +
                                    std::pow(dz * resolution, 2));
            
            if (dist <= separation) {
              astar::Cell expanded(other_start.x() + dx, other_start.y() + dy, other_start.z() + dz);
              temp_obstacles.insert(expanded);
              other_starts_obstacles.insert(expanded);
            }
          }
        }
      }
    }
  }

  // plan the path with temporary obstacles
  std::optional<std::list<astar::Position>> path_opt = astar.plan(start, goal, temp_obstacles);

  if (path_opt) {
    for (const astar::Position &pos : path_opt.value()) {
      path.push_back(Eigen::Vector3d(pos.x(), pos.y(), pos.z()));

      // | -------- Expand obstacle region around each path point -------- |
      astar::Cell center = astar.toGrid(pos.x(), pos.y(), pos.z());
      int expand_cells = static_cast<int>(std::ceil(separation / resolution));

      for (int dx = -expand_cells; dx <= expand_cells; dx++) {
        for (int dy = -expand_cells; dy <= expand_cells; dy++) {
          for (int dz = -expand_cells; dz <= expand_cells; dz++) {

            // Convert back to physical coordinates
            double dist = std::sqrt(std::pow(dx * resolution, 2) +
                                    std::pow(dy * resolution, 2) +
                                    std::pow(dz * resolution, 2));

            if (dist <= separation) {
              astar::Cell expanded(center.x() + dx, center.y() + dy, center.z() + dz);
              obstacles.insert(expanded);  // Add to permanent obstacles
            }
          }
        }
      }
    }
  } else {
    printf("path not found for UAV %d\n", i);
    path.push_back(initial_states[i]);
    path.push_back(final_states[assignment[i]]);
  }

    paths.push_back(path);
  }

// Visualize the obstacle set for debugging
  for (const auto& obstacle : obstacles) {
    astar::Position pos = astar.fromGrid(obstacle);
    action_handlers_.visualizeCube(
      Position_t{pos.x(), pos.y(), pos.z()}, 
      Color_t{1.0, 0.5, 0.0, 0.2},  // Red, semi-transparent
      resolution
    );
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
