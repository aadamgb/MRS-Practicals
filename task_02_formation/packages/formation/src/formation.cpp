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
// *Credits: Hungarian Solve algorithm produced by an LLM


std::tuple<Eigen::Vector4d, Eigen::Matrix4d> Formation::lkfPredict(const Eigen::Vector4d &x, const Eigen::Matrix4d &x_cov){
  Eigen::Vector4d new_x;      // the updated state vector, x[k+1]
  Eigen::Matrix4d new_P;  // the updated covariance matrix

  new_x = A_ * x ;
  new_P = A_ * x_cov * A_.transpose() + Q_;

  return {new_x, new_P};
}

std::tuple<Eigen::Vector4d, Eigen::Matrix4d> Formation::lkfCorrect(const Eigen::Vector4d &x, const Eigen::Matrix4d &x_cov, const Eigen::Vector2d &meas){

  Eigen::Vector4d new_x;      // the updated state vector, x[k+1]
  Eigen::Matrix4d new_P;  // the updated covariance matrix

  // Kalman gain 
  Eigen::Matrix<double,4,2> K = x_cov * H_.transpose() *
      (H_ * x_cov * H_.transpose() + R_).ldlt().solve(Eigen::Matrix2d::Identity());

  // Updated state prediction 
  new_x = x + K * (meas - H_ * x);

  // Updated covariance (Joseph form for numerical stability)
  new_P = (Eigen::Matrix4d::Identity() - K * H_) * x_cov * 
              (Eigen::Matrix4d::Identity() - K * H_).transpose() + K * R_ * K.transpose();

  return {new_x, new_P};
}

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {

  // INITIALIZE YOUR KALMAN FILTER HERE
  A_ = Eigen::Matrix4d::Identity();  
  // P_ = Eigen::Matrix4d::Identity();  
  H_ = Eigen::Matrix<double, 2, 4>::Zero();
  x_.setZero();

  P_.setZero();
  P_(0,0) = P_(1,1) = 25.0;   // 5 m position uncertainty
  P_(2,2) = P_(3,3) = 4.0;    // 2 m/s velocity uncertainty

  Q_ = Eigen::Matrix4d::Zero();  
  R_ = Eigen::Matrix2d::Identity() * 0.25;

  double dt = 0.1; // 10 Hz
    
  A_(0,2) = dt; A_(1,3) = dt;

  H_(0,0) = 1; H_(1,1) = 1;

  double accel_var = 0.25*0.25; // (m/s^2)^2
  // Simple process noise for constant velocity:
  double dt2 = dt*dt;
  Q_(0,0) = 0.25*dt2*dt2*accel_var;
  Q_(0,2) = 0.5*dt2*accel_var;
  Q_(1,1) = Q_(0,0);
  Q_(1,3) = Q_(0,2);
  Q_(2,0) = Q_(0,2); Q_(3,1)=Q_(0,2);
  Q_(2,2) = dt2*accel_var; Q_(3,3)=Q_(2,2);

  first_iteration_ = true;

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
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(
  const std::vector<Eigen::Vector3d> &initial_states,
  const std::vector<Eigen::Vector3d> &final_states) 
{
  int n_uavs = initial_states.size();
  action_handlers_.visualizeCube(Position_t{0, 0, 0}, Color_t{0.0, 0.0, 1.0, 0.05}, 1.0);

  // | --- Step 1: Assign optimally goals to starts with the Hungarian Algorithm --- |

  // 1.1 Calculate cost matrix (Eculidean distance between start and goal 
  std::vector<std::vector<double>> cost(n_uavs, std::vector<double>(n_uavs));
  for (int i = 0; i < n_uavs; i++) {
    for (int j = 0; j < n_uavs; j++) {
      cost[i][j] = (initial_states[i] - final_states[j]).norm();
    }
  }

  // 1.2 Apply the Hungarian algorithm
  std::vector<int> assignment = hungarianSolve(cost);

  // | --- Step 2: Use A* to find paths from starts to goals --- |

  // 2.1 Initialize the algorithm and paths vectop
  const double resolution = 0.6;                         // grid resolution
  double separation = 1.2;                               // minimum distance between paths
  astar::Astar astar(resolution);                        // A* init
  std::set<astar::Cell> obstacles;                       // shared obstacle map
  std::vector<std::vector<Eigen::Vector3d>> paths;

  // 2.2 Plan paths for each UAV sequentially 
  for (int i = 0; i < n_uavs; i++) {
    std::vector<Eigen::Vector3d> path;
    
    astar::Position start(initial_states[i][0], 
                          initial_states[i][1], 
                          initial_states[i][2]);

    astar::Position goal(final_states[assignment[i]][0], 
                         final_states[assignment[i]][1], 
                         final_states[assignment[i]][2]);

    // 2.3 Mark the other starts and goals as temporary obstacles
    std::set<astar::Cell> temp_obstacles = obstacles;
    for (int j = 0; j < n_uavs; j++) {
      if (j == i) continue;
      bool is_future = j > i;

      astar::Cell start_cell = astar.toGrid(initial_states[j][0], initial_states[j][1], initial_states[j][2]);
      astar::Cell goal_cell  = astar.toGrid(final_states[assignment[j]][0], final_states[assignment[j]][1], final_states[assignment[j]][2]);

      int expand_cells = static_cast<int>(std::ceil(separation / resolution));

      for (int dx = -expand_cells; dx <= expand_cells; ++dx)
        for (int dy = -expand_cells; dy <= expand_cells; ++dy)
          for (int dz = -expand_cells; dz <= expand_cells; ++dz) {
            temp_obstacles.insert({start_cell.x()+dx, start_cell.y()+dy, start_cell.z()+dz});
            if (is_future)
              temp_obstacles.insert({goal_cell.x()+dx, goal_cell.y()+dy, goal_cell.z()+dz});
          }
    }

    // 2.4 Plan path with fixed + temporary obstacles
    std::optional<std::list<astar::Position>> path_opt = astar.plan(start, goal, temp_obstacles);
    if (path_opt) {
      for (const astar::Position &pos : path_opt.value()) {
        // Pushing found path to paths vector
        path.push_back(Eigen::Vector3d(pos.x(), pos.y(), pos.z()));     

        // Inflate current path and mark it as obstacle for next planning iterations
        astar::Cell center = astar.toGrid(pos.x(), pos.y(), pos.z());   
        int expand_cells = static_cast<int>(std::ceil(separation / resolution));
        for (int dx = -expand_cells; dx <= expand_cells; dx++) {
          for (int dy = -expand_cells; dy <= expand_cells; dy++) {
            for (int dz = -expand_cells; dz <= expand_cells; dz++) {
              astar::Cell neighbor(center.x() + dx, center.y() + dy, center.z() + dz);
              obstacles.insert(neighbor);                              
            }
          }
        }
      }
    } else {
      printf("[WARN] Path not found for UAV %d\n", i);
    }

    // | --- Step 3: Straighten path (not required) --- |
    std::vector<Eigen::Vector3d> straightened_path;
    if (!path.empty()) {
      straightened_path.push_back(path.front());
      int p = 0;
      while (p < (int)path.size() - 1) {
        int l = path.size() - 1;
        for (; l > p + 1; --l) {
          astar::Position pos_p(path[p][0], path[p][1], path[p][2]);
          astar::Position pos_l(path[l][0], path[l][1], path[l][2]);
          if (astar.hasLineOfSight(pos_p, pos_l, temp_obstacles)) {
            break;
          }
      }
        straightened_path.push_back(path[l]);
        p = l;
      }
    }

    // --- Now inflate the (straightened) path into the global obstacles set ---
    int expand_cells = static_cast<int>(std::ceil(separation / resolution));
    for (const auto &pt : straightened_path) {
      astar::Cell center = astar.toGrid(pt.x(), pt.y(), pt.z());
      for (int dx = -expand_cells; dx <= expand_cells; ++dx)
        for (int dy = -expand_cells; dy <= expand_cells; ++dy)
          for (int dz = -expand_cells; dz <= expand_cells; ++dz) {
            astar::Cell neighbor(center.x() + dx, center.y() + dy, center.z() + dz);
            obstacles.insert(neighbor);
          }
    }
    paths.push_back(straightened_path);

    // paths.push_back(path);
  }

  // Optional: visualize all obstacles at the end
  for (const auto& obstacle : obstacles) {
    astar::Position pos = astar.fromGrid(obstacle);
    action_handlers_.visualizeCube(
      Position_t{pos.x(), pos.y(), pos.z()},
      Color_t{1.0, 0.5, 0.0, 0.05},  // red transparent
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

Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions,
                                           const Eigen::VectorXd &distances) {
  const int N = static_cast<int>(positions.size());
  if (N < 3) {
    return Eigen::Vector3d::Zero();
  }

  // -------------------- Initialization --------------------
  Eigen::Vector3d s = Eigen::Vector3d::Zero();
  for (const auto &p : positions) {
    s += p;
  }
  s /= N;
  s.z() = 0.0;

  const int max_iterations = 200;
  const double damping_initial = 1e-2;    // LM damping parameter
  const double convergence_tol = 1e-6;    // stop when Δs is small
  const double position_limit = 90.0;    // clamp x, y

  double lambda = damping_initial;        // LM damping factor

  // -------------------- Iterative optimization --------------------
  for (int iter = 0; iter < max_iterations; ++iter) {
    Eigen::VectorXd g(N);               // residual vector
    Eigen::MatrixXd J(N, 3);            // Jacobian matrix

    // Compute residuals and Jacobian
    for (int i = 0; i < N; ++i) {
      Eigen::Vector3d diff = s - positions[i];
      double dist_est = diff.norm();

      if (dist_est < 1e-8) dist_est = 1e-8;

      g(i) = dist_est - distances[i];
      J.row(i) = diff.transpose() / dist_est;
    }

    // Compute LM step
    Eigen::Matrix3d H = J.transpose() * J;
    Eigen::Vector3d g_norm = J.transpose() * g;

    H += lambda * Eigen::Matrix3d::Identity();

    Eigen::Vector3d delta = -H.ldlt().solve(g_norm);
    Eigen::Vector3d s_new = s + delta;

    // Apply constraints
    s_new.x() = std::max(-position_limit, std::min(position_limit, s_new.x()));
    s_new.y() = std::max(-position_limit, std::min(position_limit, s_new.y()));
    s_new.z() = 0.0;

    // Evaluate new residual norm
    double old_error = g.squaredNorm();
    double new_error = 0.0;

    for (int i = 0; i < N; ++i) {
      new_error += std::pow((s_new - positions[i]).norm() - distances[i], 2);
    }

    // LM adaptive damping
    if (new_error < old_error) {
      // Accept step
      s = s_new;
      lambda *= 0.7;  // decrease damping (trusting model more)
    } else {
      // Reject step
      lambda *= 2.0;  // increase damping (trust model less)
    }

    // Check for convergence
    if (delta.norm() < convergence_tol) {
      break;
    }
  }

  // -------------------- Return final estimate --------------------
  return s;
}

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

  // Raw measurement from multilateration
  Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);


  if(first_iteration_){
    x_[0] = target_position[0]; x_[1] = target_position[2];
    first_iteration_ = false;
  }
  
  // // Filtered position
  Eigen::Vector2d meas(target_position[0], target_position[1]);
  std::tie(x_, P_) = lkfPredict(x_, P_);
  std::tie(x_, P_) = lkfCorrect(x_, P_, meas);

  // | ------------- maintain 5 s average (10 Hz → 50 samples) ------------ |
  static std::deque<Eigen::Vector3d> target_buffer;
  static const size_t max_samples = 50; // 50 * 10 Hz = 5s

  // target_buffer.push_back(Eigen::Vector3d(x_[0], x_[1], target_position[2]));
  target_buffer.push_back(target_position);

  if (target_buffer.size() > max_samples) {
    target_buffer.pop_front();
  }

  Eigen::Vector3d target_position_avg = Eigen::Vector3d::Zero();
  for (const auto &pos : target_buffer) {
    target_position_avg += pos;
  }

  target_position_avg /= target_buffer.size();

  // Eigen::Vector2d meas(target_position_avg[0], target_position_avg[1]);
  // std::tie(x_, P_) = lkfPredict(x_, P_);
  // std::tie(x_, P_) = lkfCorrect(x_, P_, meas);

  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);
  action_handlers.visualizeCube(Position_t{x_[0], x_[1], 0}, Color_t{0.0, 0.0, 0.0, 1.0}, 1.0);
  action_handlers.visualizeCube(Position_t{target_position_avg[0], target_position_avg[1], target_position_avg[2]}, Color_t{0.0, 1.0, 0.0, 1.0}, 1.0);
  action_handlers.visualizeCube(Position_t{formation_state.virtual_leader[0], 
                                           formation_state.virtual_leader[1], 
                                           formation_state.virtual_leader[2]}, Color_t{1.0, 0.5, 0.0, 1.0}, 1.0);

  // | ------------------- Put your code here ------------------- |
  Eigen::Vector3d distance = (target_position_avg - formation_state.virtual_leader);
  if (user_defined_variable_ != 1 && user_defined_variable_ != 2 && user_defined_variable_ != 4 &&
      user_defined_variable_ != 5 && user_defined_variable_ != 10 && user_defined_variable_ !=7) {

    static double inside_start_time = -1.0; // when we first got within 15 m

    if (distance.norm() > 15.0 && user_defined_variable_ != 6 && user_defined_variable_ != 7) {
      // reset timer since we are now outside range
      inside_start_time = -1.0;

      if (std::abs(distance.x()) > std::abs(distance.y())) {
        user_defined_variable_ = 0;
      } else {
        user_defined_variable_ = 3;
      }

    } else if (distance.norm() <= 12.0) {
      // started being inside the 15 m zone
      if (inside_start_time < 0.0) {
        inside_start_time = time_stamp; // record the time when we entered the zone
      }

      // check how long we have stayed inside
      if (time_stamp - inside_start_time >= 10.0) {
        printf("Distance to target is: %0.2f \n", distance.norm());
        user_defined_variable_ = 6;
      }
    }
  }
  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }

  // this is an example of a "state machine"
  switch (user_defined_variable_) {

    // case state_:
    case 0: {
      std::vector<Eigen::Vector3d> formation_line;
      formation_line.push_back(Eigen::Vector3d(-3.0, 0.5, 1.0));
      formation_line.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));
      formation_line.push_back(Eigen::Vector3d(3.0, -0.5, 6.0));

      // plan paths to reshape the formation
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      } else {
         printf("Reshaping to go HORIZONTALLY\n");
         h_= true;
         v_ = false;
      }

      user_defined_variable_ = (distance.x() > 0) ? 1 : 2;

      break;
    }

    case 1: {
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(std::min(formation_state.virtual_leader[0] + 20, 90.0), 
                                                                       formation_state.virtual_leader[1], 3));
      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
         printf("Going RIGHT\n");
      }
      
      user_defined_variable_ = 9;

      break;
    }

    case 2: {

      // tell the virtual leader to move to the center of the arena
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(std::max(formation_state.virtual_leader[0] - 20, -90.0), 
                                                                       formation_state.virtual_leader[1], 3));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
         printf("Going LEFT\n");
      }

      user_defined_variable_ = 9;

      break;
    }

    case 3: {
      std::vector<Eigen::Vector3d> formation_line;
      formation_line.push_back(Eigen::Vector3d(-0.5, 3.0, 2.0));
      formation_line.push_back(Eigen::Vector3d(0.0, 0.0, 4.0));
      formation_line.push_back(Eigen::Vector3d(0.5, -3.0, 6.0));

      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      } else {
         printf("Reshaping to go VERTICALLY\n");
         v_ = true;
         h_ = false;
      }

      user_defined_variable_ = (distance.y() > 0) ? 4 : 5;

      break;
    }

    case 4: {
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(formation_state.virtual_leader[0], 
                                                                       std::min(formation_state.virtual_leader[1] + 20, 90.0), 3));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
         printf("Going UP\n");
      }

      user_defined_variable_ = 9;

      break;
    }


    case 5: {

      // tell the virtual leader to move to the center of the arena
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(formation_state.virtual_leader[0], 
                                                                       std::max(formation_state.virtual_leader[1] - 20, -90.0) , 3));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
         printf("Going DOWN\n");
      }

      user_defined_variable_ = 9;

      break;
    }

    case 6: {

      printf("HOLA!\n");
      std::vector<Eigen::Vector3d> formation_line;
      formation_line.push_back(Eigen::Vector3d(-0.5, 0.5, 3.5));
      formation_line.push_back(Eigen::Vector3d(0.0, 0.0, 0.5));
      formation_line.push_back(Eigen::Vector3d(0.5, -0.5, 7.0));

      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(formation_state.followers, formation_line);

      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
         printf("Reshaping to CHASE\n");
      }

      user_defined_variable_ = 10;

      break;
    }

    case 7: {

      printf("Going back to nearest cell center!\n");

      // Get current virtual leader position
      Eigen::Vector3d current_pos = formation_state.virtual_leader;

      // Find nearest cell center (grid spacing = 10 m)
      // Each cell center is at (10a, 10b), where a,b ∈ {-9,...,9}
      int nearest_a = static_cast<int>(std::round(current_pos.x() / 10.0));
      int nearest_b = static_cast<int>(std::round(current_pos.y() / 10.0));

      // Clamp indices to the allowed range [-9, 9]
      nearest_a = std::max(-9, std::min(9, nearest_a));
      nearest_b = std::max(-9, std::min(9, nearest_b));

      // Compute nearest center position (keeping altitude = 3 m)
      Eigen::Vector3d nearest_center(10.0 * nearest_a, 10.0 * nearest_b, 3.0);

      printf("Moving to cell center at (%.2f, %.2f, %.2f)\n",
            nearest_center.x(), nearest_center.y(), nearest_center.z());

      bool success = action_handlers.setLeaderPosition(nearest_center);
      if (!success) {
        printf("Something went wrong moving the leader to the nearest center.\n");
        return;
      } else {
        printf("Leader reached nearest cell center.\n");
      }

      // Once centered, reset or transition to another state
      user_defined_variable_ = 10;
      break;
    }

    default: {

      bool success = false;


      double dx = target_position_avg[0] - formation_state.virtual_leader[0];
      double dy = target_position_avg[1] - formation_state.virtual_leader[1];

      if (std::abs(dx) > std::abs(dy) && !v_) {
          h_ = true;
          v_ = false;
          success = action_handlers.setLeaderPosition(Eigen::Vector3d(target_position_avg[0], 
                                                                       formation_state.virtual_leader[1] , 3));
      } else if(std::abs(dx) < std::abs(dy) && !h_) {
          v_= true;
          h_ = false;
          success = action_handlers.setLeaderPosition(Eigen::Vector3d(formation_state.virtual_leader[0], 
                                                                       target_position_avg[1] , 3));
      } else {
        if(std::abs(dx) > std::abs(dy)){
          h_ = true;
          v_ = false;
        } else {
          h_ = false;
          v_ = true;
        }
        user_defined_variable_ = 7;

        break;
      }


      // bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(target_position_avg[0], target_position_avg[1], 0));

      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      } else {
      printf("Chasing the target!\n");
      }

      user_defined_variable_ =  (distance.norm() < 10.0) ? 10: 7;
      // user_defined_variable_ = 7;
      break;
    }
  }
}

//}

}  // namespace task_02_formation
