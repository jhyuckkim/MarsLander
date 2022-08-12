// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    // Activate attitude stabilization if off
    if (!stabilized_attitude) stabilized_attitude = true;
    
    // Define variables
    double Kh, scale_factor, h, descending_rate, ground_speed, Kp, vertical_e, vertical_P_out, delta, vertical_throttle, horizontal_throttle;
    
    // Calculate vertical error term
    Kh = 0.04; // This contant controls auto-pilot in low altitude increase this constant for faster landing
    scale_factor = 800; // Decrease this factor for safer landing from high altitude
    h = position.abs() - MARS_RADIUS;
    descending_rate = velocity * position / position.abs();
    vertical_e = -(0.5 + scale_factor * log(Kh / scale_factor * h + 1) + descending_rate);
    /*
     I realized that if I use a linear relation between decreasing rate and altitude as a target, then the point where the error term becomes positive is too late for landings from high altitude and the lander crashes even after having maximum throttle ever since the error term becomes positive.
     To handle landings from high altitude, I have tried decreasing the value of Kh but this made low altitude landing too slow.
     To I used a natural log relationship between decreasing rate and altitude as a target, so that the point where the error term becomes positive will be eariler for high altitude landings while keeping the gradient of the traget graph for low altitudes.
     Such target relationship allows me to do autopilot landings from all the scenarios.
     */
    
    /*
    // Save h and descending rate to text file
    ofstream fout;
    fout.open("/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/lander/assignment5.txt", ios::app);
    if (fout) { // file opened successfully
        fout << h << ' ' << descending_rate << endl;
    }
    else { // file did not open successfully
      cout << "Could not open trajectory file for writing" << endl;
    }
    */
    
    // Calculate vertical P out
    Kp = 0.8;
    vertical_P_out = Kp * vertical_e;
    
    // Calculate vertical throttle
    delta = 0.5;
    if (vertical_P_out <= -delta) vertical_throttle = 0;
    else if(vertical_P_out < 1 - delta) vertical_throttle = delta + vertical_P_out;
    else vertical_throttle = 1;
    
    ground_speed = (velocity ^ position).abs() / position.abs();
    
    // If in orbit, backfire
    if (- GRAVITY * MARS_MASS / position.abs() + 0.5 * pow(velocity.abs(), 2) > - GRAVITY * MARS_MASS / (2 * (EXOSPHERE / 2 + MARS_RADIUS)) && ground_speed > 0.7 * sqrt(GRAVITY * MARS_MASS / position.abs())) {
        horizontal_throttle = 1;
    }
    else{
        // define variables
        double horizontal_e, horizontal_P_out, Kh_horizontal;
        
        // Calculate horizontal error term
        Kh_horizontal = 0.02;
        ground_speed = (velocity ^ position).abs() / position.abs();
        horizontal_e = ground_speed - Kh_horizontal * h;
        
        // Calculate horizontal P out
        horizontal_P_out = Kp * horizontal_e;
        
        // Calculate horizontal throttle
        if (horizontal_P_out <= 0) horizontal_throttle = 0;
        else if(horizontal_P_out < 1) horizontal_throttle = horizontal_P_out;
        else horizontal_throttle = 1;
    }
    
    // Calculate throttle
    if (sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2)) <= 1) throttle = sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2));
    else throttle = 1;
    
    // Calculate angle
    int angle;
    if (vertical_throttle == 0 && horizontal_throttle != 0) angle = 90;
    else angle = (int) atan(horizontal_throttle / vertical_throttle) * 180 / M_PI;
    stabilized_attitude_angle = -angle;
    if (get_closeup_coords_right() * velocity < 0) stabilized_attitude_angle = -stabilized_attitude_angle;
    
    // Deploy parachute
    if (vertical_e > 2.5 && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() == true && position.abs() < MARS_RADIUS + EXOSPHERE) parachute_status = DEPLOYED;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    // State static variable and store position in temp
    static vector3d previous_position;
    vector3d temp = position;
        
    // Calculate mass
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    
    // Calculate net force
    vector3d thrust_force = thrust_wrt_world();
    vector3d weight_force = (- GRAVITY * MARS_MASS * mass / pow(position.abs(),3)) * position;
    vector3d lander_drag_force = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * pow(LANDER_SIZE, 2) * M_PI * velocity.abs() * velocity;
    vector3d chute_drag_force = -0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * 5 * pow((2.0 *LANDER_SIZE), 2) * velocity.abs() * velocity;
    vector3d drag_force;
    if (parachute_status == DEPLOYED) {
        drag_force = lander_drag_force + chute_drag_force;
    }
    else {
        drag_force = lander_drag_force;
    }
    vector3d net_force = thrust_force + weight_force + drag_force;
    
    // Calculate acceleration
    vector3d acceleration = net_force / mass;
    
    // Execute Euler integration when initializing
    if (simulation_time == 0.0) {
        position = position + delta_t * velocity;
        velocity = velocity + delta_t * acceleration;
    }
    // Verlet integration
    else {
        position = 2 * position - previous_position + acceleration * pow(delta_t, 2);
        velocity = (position - temp) / delta_t;
    }
    
    previous_position = temp;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "areostationary orbit";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    // an areostationary orbit
    position = vector3d(cbrt(GRAVITY * MARS_MASS * pow(MARS_DAY, 2) / (4 * pow(M_PI, 2))), 0.0, 0.0);
    velocity = vector3d(0.0, cbrt(GRAVITY * MARS_MASS * 2 * M_PI / MARS_DAY));
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
