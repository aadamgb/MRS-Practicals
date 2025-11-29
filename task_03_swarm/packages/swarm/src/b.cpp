std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  // Setup the output action
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  
  // Vectors for Boids rules
  Eigen::Vector3d v_separation = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_alignment = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_cohesion = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_target = state.target;

  // Weights (Mapping user_params to readable names)
  // Assuming param1=Separation, param2=Alignment, param3=Cohesion, param4=Target
  // You can adjust these mappings based on your specific GUI/config setup
  double w_separation = user_params.param1; 
  double w_alignment  = user_params.param2;
  double w_cohesion   = user_params.param3;
  double w_target     = user_params.param4;

  // Hard safety limit
  const double MIN_DIST_LIMIT = 0.3;
  const double SAFETY_BUFFER = 0.05; // React slightly before hitting the limit
  bool collision_risk = false;

  int neighbor_count = 0;
  Eigen::Vector3d average_neighbor_pos = Eigen::Vector3d::Zero();

  // Iterate over the states of the visible neighbors
  for (const auto &n_state : state.neighbors_states) {
    auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    double dist = n_pos_rel.norm();

    // Prevent division by zero
    if (dist < 0.001) dist = 0.001;

    // --- 1. CRITICAL SAFETY CHECK (The 0.3 Constraint) ---
    // If we are within the limit (plus a tiny buffer), we must abort standard boids
    // and strictly evade.
    if (dist < (MIN_DIST_LIMIT + SAFETY_BUFFER)) {
        collision_risk = true;
        // Create a massive repulsive force vector directly away from this neighbor
        // The closer we are, the stronger the force (exponential)
        Eigen::Vector3d emergency_repulsion = -n_pos_rel.normalized() / (dist * dist);
        v_separation += emergency_repulsion;
    }

    // --- Standard Boids Accumulation ---
    if (!collision_risk) {
        // Separation: Move away from neighbors (weighted by inverse distance)
        v_separation += -n_pos_rel.normalized() / dist;

        // Alignment: Accumulate neighbor velocities
        v_alignment += n_vel_global;

        // Cohesion: Accumulate relative positions
        average_neighbor_pos += n_pos_rel;
    }
    
    neighbor_count++;
  }

  // Finalize Calculation
  if (collision_risk) {
      // PRIORITY MODE: Ignore Alignment, Cohesion, and Target.
      // Only apply maximum separation force to guarantee the 0.3 limit.
      action = v_separation * 500.0; // Hard multiplier to ensure saturation of max velocity
      
      // Visual debugging for collision risk (Red arrow)
      action_handlers.visualizeArrow("separation_emergency", action.normalized()*10, Color_t{1.0, 0.0, 0.0, 1.0});
  
  } else if (neighbor_count > 0) {
      // Normalize Alignment (average direction)
      v_alignment.normalize();

      // Normalize Cohesion (vector towards center of mass)
      v_cohesion = (average_neighbor_pos / neighbor_count).normalized();
      
      // Calculate total weighted action
      action = (w_separation * v_separation) + 
               (w_alignment * v_alignment) + 
               (w_cohesion * v_cohesion) + 
               (w_target * v_target);
               
      // Visual debugging
      action_handlers.visualizeArrow("separation", v_separation.normalized(), Color_t{1.0, 0.0, 0.0, 1.0}); // Red
      action_handlers.visualizeArrow("alignment", v_alignment.normalized(), Color_t{0.0, 1.0, 0.0, 1.0});  // Green
      action_handlers.visualizeArrow("cohesion", v_cohesion.normalized(), Color_t{0.0, 0.0, 1.0, 1.0});    // Blue
  } else {
      // No neighbors? Just seek the target.
      action = v_target * w_target;
  }

  // Maintain current distribution
  Distribution my_distribution = state.distribution;
  if(state.nearby_beacon) {
    my_distribution = state.beacon_distribution;
  } else {
    my_distribution.set(1, 1.0);
  }

  return {action, my_distribution};
}