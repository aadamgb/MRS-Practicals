#!/bin/bash

# Check if the multiplier input is provided
if [ -z "$1" ]; then
  echo "Specify the realtime-factor"
  exit 1
fi

MULTIPLIER=$1

# Get the current physics properties
output=$(rosservice call /gazebo/get_physics_properties)

# Extract other necessary properties to keep them unchanged
gravity_x=$(echo "$output" | grep "gravity:" -A 3 | grep "x:" | awk '{print $2}')
gravity_y=$(echo "$output" | grep "gravity:" -A 3 | grep "y:" | awk '{print $2}')
gravity_z=$(echo "$output" | grep "gravity:" -A 3 | grep "z:" | awk '{print $2}')

# Extract ode_config parameters
auto_disable_bodies=$(echo "$output" | grep "auto_disable_bodies:" | awk '{print $2}')
sor_pgs_precon_iters=$(echo "$output" | grep "sor_pgs_precon_iters:" | awk '{print $2}')
sor_pgs_iters=$(echo "$output" | grep "sor_pgs_iters:" | awk '{print $2}')
sor_pgs_w=$(echo "$output" | grep "sor_pgs_w:" | awk '{print $2}')
sor_pgs_rms_error_tol=$(echo "$output" | grep "sor_pgs_rms_error_tol:" | awk '{print $2}')
contact_surface_layer=$(echo "$output" | grep "contact_surface_layer:" | awk '{print $2}')
contact_max_correcting_vel=$(echo "$output" | grep "contact_max_correcting_vel:" | awk '{print $2}')
cfm=$(echo "$output" | grep "cfm:" | awk '{print $2}')
erp=$(echo "$output" | grep "erp:" | awk '{print $2}')
max_contacts=$(echo "$output" | grep "max_contacts:" | awk '{print $2}')

# Calculate the new max_update_rate
new_rate=$(echo "$MULTIPLIER * 250" | bc)
time_step=0.004

# Call the set_physics_properties service with the new max_update_rate
rosservice call /gazebo/set_physics_properties "{
  time_step: $time_step,
  max_update_rate: $new_rate,
  gravity: {x: $gravity_x, y: $gravity_y, z: $gravity_z},
  ode_config: {
    auto_disable_bodies: $auto_disable_bodies,
    sor_pgs_precon_iters: $sor_pgs_precon_iters,
    sor_pgs_iters: $sor_pgs_iters,
    sor_pgs_w: $sor_pgs_w,
    sor_pgs_rms_error_tol: $sor_pgs_rms_error_tol,
    contact_surface_layer: $contact_surface_layer,
    contact_max_correcting_vel: $contact_max_correcting_vel,
    cfm: $cfm,
    erp: $erp,
    max_contacts: $max_contacts
  }
}"
