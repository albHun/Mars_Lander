// Mars lander simulator
// Version 1.9
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <fstream>

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
	double threshold = 0.9;
	double kp = 10000, kh = 0.0187;
	double pout = -kp * (0.5 + kh * (position.abs()-MARS_RADIUS) - velocity.abs());
	if (pout <= -threshold) {
		throttle = 0;
	}
	else if (pout >= 1 - threshold) {
		throttle = 1;
	}
	else {
		throttle = threshold + pout;
	}
		
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
	
	vector3d gravity = - GRAVITY * MARS_MASS * (UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY) * position.norm() / position.abs2();
	vector3d thrust = thrust_wrt_world() * throttle;
	double air_density = atmospheric_density(position);
	vector3d drag;
	switch (parachute_status) {
		case 0: drag = -0.5 * air_density * DRAG_COEF_LANDER * LANDER_SIZE * LANDER_SIZE * M_PI * velocity.abs2() * velocity.norm();
		case 1: drag = -0.5 * air_density * DRAG_COEF_LANDER * LANDER_SIZE * LANDER_SIZE * M_PI * velocity.abs2() * velocity.norm()
					   -0.5 * air_density * DRAG_COEF_CHUTE * 5 * (2 * LANDER_SIZE) * (2 * LANDER_SIZE) * velocity.abs2() * velocity.norm();
		case 2: drag = -0.5 * air_density * DRAG_COEF_LANDER * LANDER_SIZE * LANDER_SIZE * M_PI * velocity.abs2() * velocity.norm();
	}
	vector3d net_force = gravity + thrust + drag;
	vector3d acceleration = net_force / (UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY);
	position += velocity * delta_t;
	velocity += acceleration * delta_t;
	double centri_v = velocity * position / position.abs();
	ofstream fout;
	fout.open("Height_and_centripetal_velocity.txt", ios::app);
	fout<<"Heihgt: "<<position.abs()-MARS_RADIUS<<" and centripetal velocity: "<< centri_v<<endl;
	
	
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
  scenario_description[6] = "hyperbolic escape from the edge of the exosphere";
  scenario_description[7] = "a descent from rest at 10km altitude with parachute deployed";
  scenario_description[8] = "a descent from rest at 10km altitude with autopilot";
  scenario_description[9] = "a descent from rest at the edge of the exosphere with autopilot";
	
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
	// hyperbolic escape from the edge of the exosphere
	position = vector3d(0.0, 0.0, MARS_RADIUS + EXOSPHERE);
	velocity = vector3d(0.0, 5000.0, 0);
	orientation = vector3d(0.0, 0.0, 0.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = false;
	autopilot_enabled = false;
	break;

  case 7:
	// a descent from rest at 10km altitude with parachute deployed
	position = vector3d(0.0, -(MARS_RADIUS + 10000), 0.0);
	velocity = vector3d(0.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = false;
	break;

  case 8:
	// a descent from rest at 10km altitude with autopilot
	position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
	velocity = vector3d(0.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = true;
	break;

  case 9:
	// a descent from rest at the edge of the exosphere with autopilot
	position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
	velocity = vector3d(0.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = true;
	break;

  }
}
