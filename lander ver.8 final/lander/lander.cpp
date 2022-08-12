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
{
    // Activate attitude stabilization if off
    if (!stabilized_attitude) stabilized_attitude = true;
    
    // Define variables
    double Kh, scale_factor, h, descending_rate, Kp, vertical_e, horizontal_e, vertical_P_out, delta, vertical_throttle, horizontal_throttle;
    
    // Calculate average velocity of last twenty velocities
    vector3d avg_velocity = vector3d(0, 0, 0);
    for (int i=0; i<20; i++) {
        avg_velocity += last_20_velocity[i];
    }
    avg_velocity = avg_velocity / 20;
    
    // Calculate expected position & velocity after (ENGINE_LAG + ENGINE_DELAY)
    vector3d expected_position = position + (engine_lag * 1.2 + engine_delay) * avg_velocity;
    vector3d expected_velocity = velocity + (engine_lag * 1.2 + engine_delay) * acceleration;
    
    // Calculate vertical error term
    Kh = 0.04; // This contant controls auto-pilot in low altitude increase this constant for faster landing
    scale_factor = 800; // Decrease this factor for safer landing from high altitude
    h = expected_position.abs() - MARS_RADIUS;
    descending_rate = expected_velocity * expected_position / expected_position.abs();
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
    fout.open("/Users/junhyuckkim/OneDrive - University of Cambridge/Part IA/1CW Mars Lander/lander ver.7 wind pattern/scenario5.txt", ios::app);
    if (fout) { // file opened successfully
        fout << position.abs()-MARS_RADIUS << ' ' << velocity * position / position.abs() << endl;
    }
    else { // file did not open successfully
      cout << "Could not open trajectory file for writing" << endl;
    }
    */
    // Calculate vertical P out
    Kp = 0.25;
    vertical_P_out = Kp * vertical_e;
    
    // Calculate vertical throttle
    delta = 0.6;
    if (vertical_P_out <= -delta) vertical_throttle = 0;
    else if(vertical_P_out < 1 - delta) vertical_throttle = delta + vertical_P_out;
    else vertical_throttle = 1;
    
    // If in orbit, backfire
    bool backfire = false;
    if (- GRAVITY * MARS_MASS / expected_position.abs() + 0.5 * pow(expected_velocity.abs(), 2) > - GRAVITY * MARS_MASS / (2 * (EXOSPHERE / 2 + MARS_RADIUS)) && ((expected_velocity ^ expected_position).abs() / expected_position.abs()) > 0.7 * sqrt(GRAVITY * MARS_MASS / expected_position.abs())) {
        backfire = true;
        horizontal_throttle = 1;
        horizontal_e = 0;
        if (position.abs() > cbrt(GRAVITY * MARS_MASS * pow(MARS_DAY, 2) / (4 * pow(M_PI, 2)))) stabilized_attitude_angle = -stabilized_attitude_angle;
    }
    else{
        // define variables
        double horizontal_P_out, Kh_horizontal, Kp_horizontal;
        
        // Calculate horizontal error term
        Kh_horizontal = 0.02;
        horizontal_e = ground_speed - Kh_horizontal * h;
        
        // Calculate horizontal P out
        Kp_horizontal = 0.8;
        horizontal_P_out = Kp_horizontal * horizontal_e;
        
        // Calculate horizontal throttle
        if (horizontal_P_out <= 0) horizontal_throttle = 0;
        else if(horizontal_P_out < 1) horizontal_throttle = horizontal_P_out;
        else horizontal_throttle = 1;
        if (vertical_e > 200) horizontal_throttle = 0;
    }
    
    // Calculate throttle
    if (sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2)) <= 1) throttle = sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2));
    else throttle = 1;
    
    // Calculate angle
    if (throttle != 0.0) {
        int angle;
        if (vertical_throttle == 0 && horizontal_throttle != 0) angle = 90;
        else angle = (int) (atan(horizontal_throttle / vertical_throttle) * 180 / M_PI);
        stabilized_attitude_angle = -angle;
        if (get_closeup_coords_right() * (velocity - (vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position)) < 0 && !backfire) stabilized_attitude_angle = -stabilized_attitude_angle;
    }
    
    // Deploy parachute
    if ((horizontal_e > 1.5 || vertical_e > 1.5) && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() == true && position.abs() < MARS_RADIUS + EXOSPHERE && !auto_parachute_disabled && simulation_time != 0.0) parachute_status = DEPLOYED;
}

void autopilot_orbital_injection (double ag, double pg)
// Autopilot to inject the lander into an orbit of desired altitude
{
    // Activate attitude stabilization if off
    if (!stabilized_attitude) stabilized_attitude = true;
    
    // Define variables
    double Kh, v, vertical_speed, Kp, vertical_e, vertical_P_out, vertical_throttle, horizontal_throttle, horizontal_speed, horizontal_e, horizontal_P_out, Kh_horizontal;
    
    // Circular orbit injection when apogee = perigee
    if (ag == pg) {
        // Calculate target speed
        v = sqrt(GRAVITY * MARS_MASS / ag);
        
        // Do nothing if already in the wanted orbit
        // if (position.abs() > r - 5 && position.abs() < r + 5  && velocity.abs() > v - 5 && velocity.abs() < v + 5 && abs(position.norm() * velocity.norm()) < 0.001) return;
        
        // Calculate vertical error term
        Kh = 0.003; // Increase this constant for faster injection
        vertical_speed = velocity * position / position.abs();
        vertical_e = Kh * (ag - position.abs()) - vertical_speed;
        
        // Calculate vertical P out
        Kp = 0.8;
        vertical_P_out = Kp * vertical_e;
        
        // Calculate vertical throttle
        if (vertical_P_out <= -1) vertical_throttle = -1;
        else if(vertical_P_out < 1) vertical_throttle = vertical_P_out;
        else vertical_throttle = 1;
        
        // Calculate horizontal error term
        Kh_horizontal = 0.005;
        horizontal_speed = (velocity ^ position).abs() / position.abs();
        horizontal_e = Kh_horizontal * (v - horizontal_speed);
        
        // Calculate horizontal P out
        horizontal_P_out = Kp * horizontal_e;
        
        // Calculate horizontal throttle
        if (horizontal_P_out <= -1) horizontal_throttle = -1;
        else if(horizontal_P_out < 1) horizontal_throttle = horizontal_P_out;
        else horizontal_throttle = 1;
    }
    else {
        // Target apogee
        if (position.abs() > (ag + pg)/2) {
            // Calculate target speed at apogee
            v = sqrt(2*GRAVITY*MARS_MASS*(1/ag-1/pg)/(1-pow(ag/pg,2)));
            
            // Calculate vertical error term
            Kh = 0.003; // Increase this constant for faster injection
            vertical_speed = velocity * position / position.abs();
            vertical_e = Kh * (ag - position.abs()) - vertical_speed;
            
            // Calculate vertical P out
            Kp = 0.8;
            vertical_P_out = Kp * vertical_e;
            
            // Calculate vertical throttle
            if (vertical_P_out <= -1) vertical_throttle = -1;
            else if(vertical_P_out < 1) vertical_throttle = vertical_P_out;
            else vertical_throttle = 1;
            
            // Calculate horizontal error term
            Kh_horizontal = 0.005;
            horizontal_speed = (velocity ^ position).abs() / position.abs();
            horizontal_e = Kh_horizontal * (v - horizontal_speed);
            
            // Calculate horizontal P out
            horizontal_P_out = Kp * horizontal_e;
            
            // Calculate horizontal throttle
            if (horizontal_P_out <= -1) horizontal_throttle = -1;
            else if(horizontal_P_out < 1) horizontal_throttle = horizontal_P_out;
            else horizontal_throttle = 1;
            
            // Stop autopilot once injected into epliptical orbit
            if (horizontal_speed > v*0.99 && horizontal_speed < v*1.01 && position.abs() > ag*0.99 && position.abs() < ag*1.01 && abs(vertical_speed) < 0.25) {
                autopilot_orbital_injection_enabled = false;
                throttle = 0;
                return;
            }
        }
        // Target perigee
        else {
            // Calculate target speed at perigee
            v = sqrt(2*GRAVITY*MARS_MASS*(1/ag-1/pg)/(pow(pg/ag,2)-1));

            // Calculate vertical error term
            Kh = 0.003; // Increase this constant for faster injection
            vertical_speed = velocity * position / position.abs();
            vertical_e = Kh * (pg - position.abs()) - vertical_speed;
            
            // Calculate vertical P out
            Kp = 0.8;
            vertical_P_out = Kp * vertical_e;
            
            // Calculate vertical throttle
            if (vertical_P_out <= -1) vertical_throttle = -1;
            else if(vertical_P_out < 1) vertical_throttle = vertical_P_out;
            else vertical_throttle = 1;
            
            // Calculate horizontal error term
            Kh_horizontal = 0.005;
            horizontal_speed = (velocity ^ position).abs() / position.abs();
            horizontal_e = Kh_horizontal * (v - horizontal_speed);
            
            // Calculate horizontal P out
            horizontal_P_out = Kp * horizontal_e;
            
            // Calculate horizontal throttle
            if (horizontal_P_out <= -1) horizontal_throttle = -1;
            else if(horizontal_P_out < 1) horizontal_throttle = horizontal_P_out;
            else horizontal_throttle = 1;
            
            // Stop autopilot once injected into epliptical orbit
            if (horizontal_speed > v*0.99 && horizontal_speed < v*1.01 && position.abs() > pg*0.99 && position.abs() < pg*1.01 && abs(vertical_speed) < 0.25) {
                autopilot_orbital_injection_enabled = false;
                throttle = 0;
                return;
            }
        }
    }
    
    // Calculate throttle
    if (sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2)) <= 1) throttle = sqrt(pow(vertical_throttle, 2) + pow(horizontal_throttle, 2));
    else throttle = 1;

    // Calculate angle
    if (vertical_throttle == 0 && horizontal_throttle > 0) stabilized_attitude_angle = 90;
    else if (vertical_throttle == 0 && horizontal_throttle < 0) stabilized_attitude_angle = -90;
    else {
        stabilized_attitude_angle = (int) (atan(horizontal_throttle / vertical_throttle) * 180 / M_PI);
        if (vertical_throttle < 0) stabilized_attitude_angle += 180;
    }
    if (get_closeup_coords_right() * (velocity - (vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position)) < 0) stabilized_attitude_angle = -stabilized_attitude_angle;
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    if (simulation_time == 0) {
        for (int i=0; i<20; i++) {
            last_20_velocity[i] = velocity;
        }
    }
    else {
        for (int i=0; i<19; i++) {
            last_20_velocity[i] = last_20_velocity[i + 1];
        }
        last_20_velocity[19] = velocity;
    }
    
    // State static variable and store position in temp
    static vector3d previous_position;
    vector3d temp = position;
        
    // Calculate mass
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    
    // Find relative velocity of lander compared to rotating Mars
    vector3d relative_velocity = velocity - (vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position);
    
    // Calculate net force
    vector3d thrust_force = thrust_wrt_world();
    vector3d weight_force = (- GRAVITY * MARS_MASS * mass / pow(position.abs(),3)) * position;
    vector3d lander_drag_force = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * pow(LANDER_SIZE, 2) * M_PI * relative_velocity.abs() * relative_velocity;
    vector3d chute_drag_force = -0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * 5 * pow((2.0 *LANDER_SIZE), 2) * relative_velocity.abs() * relative_velocity;
    vector3d drag_force;
    
    // Calculate wind force
    vector3d wind_force;
    
    // Calculate latitude
    latitude = 90.0 - acos(vector3d(0.0, 0.0, 1.0)*position.norm())*180/M_PI;
    
    if (position.abs() > MARS_RADIUS + EXOSPHERE) { wind_force = vector3d(0.0, 0.0, 0.0); wind_speed = 0.0; gusting_wind_on = false; }
    else {
        // Assume that cross-sectional area is the same for every angle
        int area;
        if (parachute_status == DEPLOYED) area = 5 * pow((2.0 *LANDER_SIZE), 2);
        else area = pow(LANDER_SIZE, 2) * M_PI;
        
        // Randomly generate wind speed within the range of plus/minus 40% of average wind speed
        wind_speed = average_wind_speed + (((rand() % (int) (average_wind_speed * 8 + 1)) - average_wind_speed * 4) / 10);
        
        // Model random gusting wind
        static int gusting_wind;
        if (gusting_wind > 0) {
            wind_speed *= 1.5; // x1.5 wind speed when gusting wind
            gusting_wind --;
            gusting_wind_on = true;
        }
        else {
            gusting_wind_on = false;
            // Certain percentage of experiencing gusting wind
            if (rand()%500==0) {
                gusting_wind = rand()%171 + 30; // Gusting wind lasts for 3~20 seconds
            }
        }

        // Calculate magnitude of wind force
        double force = 0.5 * atmospheric_density(position) * pow(wind_speed, 2) * area;
        
        // Set x and y coordinate system for horizontal wind force
        vector3d x = (vector3d(0.0, 0.0, 1.0) ^ position.norm()).norm();
        vector3d y = position.norm() ^ x;
        
        // If at polar positions, manually set x and y
        if (x == vector3d(0.0, 0.0, 0.0)) {
            x = vector3d(1.0, 0.0, 0.0);
            y = vector3d(0.0, 1.0, 0.0);
        }
        
        // Modelling wind pattern of Mars
        if (latitude > 30 && latitude < 60) {
            wind_direction = 45 + rand() % 91 - 45; // Southwest wind plus/minus 45 degrees for random wind direction
        }
        else if (latitude > 0) {
            wind_direction = -135 + rand() % 91 - 45; // Northeast wind plus/minus 45 degrees for random wind direction
        }
        else if (latitude > -30) {
            wind_direction = -45 + rand() % 91 - 45; // Southeast wind plus/minus 45 degrees for random wind direction
        }
        else if (latitude > -60) {
            wind_direction = 135 + rand() % 91 - 45; // Northwest wind plus/minus 45 degrees for random wind direction
        }
        else {
            wind_direction = rand() % 360; // Randomly generate an angle
        }
        
        // Calculate wind force using x and y coordinate system
        wind_force = force * sin(wind_direction * M_PI / 180) * x + force * cos(wind_direction * M_PI / 180) * y;
    }
    
    if (parachute_status == DEPLOYED) {
        drag_force = lander_drag_force + chute_drag_force + wind_force;
    }
    else {
        drag_force = lander_drag_force + wind_force;
    }
    vector3d net_force = thrust_force + weight_force + drag_force;
    
    // Calculate acceleration
    acceleration = net_force / mass;
    
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
    
  // Applly autopilot orbital injection
  if (autopilot_orbital_injection_enabled) autopilot_orbital_injection(apogee, perigee);

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
  scenario_description[7] = "2001 Mars Odyssey orbit";
  scenario_description[8] = "circular orbit at 30000km altitude";
  scenario_description[9] = "descent from 5km at 100km/s groundspeed";

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
    autopilot_orbital_injection_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position;
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    autopilot_orbital_injection_enabled = false;
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
    autopilot_orbital_injection_enabled = false;
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
    autopilot_orbital_injection_enabled = false;
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
    autopilot_orbital_injection_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0) + vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position;
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    autopilot_orbital_injection_enabled = false;
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
    autopilot_orbital_injection_enabled = false;
    break;

  case 7:
    // 2001 Mars Odyssey orbit
    position = vector3d(MARS_RADIUS+400000, 0.0, 0.0);
    velocity = vector3d(0.0, sqrt(GRAVITY*MARS_MASS/(MARS_RADIUS+400000))*cos(93.064*M_PI/180), sqrt(GRAVITY*MARS_MASS/(MARS_RADIUS+400000))*sin(93.064*M_PI/180));
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    autopilot_orbital_injection_enabled = false;
    break;

  case 8:
    // a circular orbit at 30000km altitude
    position = vector3d(MARS_RADIUS+30000000, 0.0, 0.0);
    velocity = vector3d(0.0, sqrt(GRAVITY*MARS_MASS/(MARS_RADIUS+30000000)), 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    autopilot_orbital_injection_enabled = false;
    break;
          
  case 9:
    // a descent from 5km atltidue at 100km/s groundspeed
    position = vector3d(0.0, -(MARS_RADIUS + 5000.0)/sqrt(2.0), (MARS_RADIUS + 5000.0)/sqrt(2.0));
    velocity = vector3d(100000.0/sqrt(2.0), 0.0, 0.0) + (vector3d(0.0, 0.0, 2 * M_PI / MARS_DAY)^position);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    autopilot_orbital_injection_enabled = false;
    break;
  }
}
