//This include guard prevents the header file being included more than once
//This is considered good practice therefore
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#ifndef VARIABLES_H
#define VARIABLES_H
//Define devices
//Initialise devices
extern pros::Controller controller;	
extern pros::Motor topLeft;
extern pros::Motor topRight;
extern pros::Motor bottomLeft;
extern pros::Motor bottomRight;

extern pros::Motor_Group left_motors;
extern pros::Motor_Group right_motors;

extern pros::Motor intake;
extern pros::Motor catapultLeft;
extern pros::Motor catapultRight;
extern pros::Motor_Group catapult_motors;
extern pros::Motor hang;
extern pros::Motor flywheel;
extern pros::ADIDigitalOut wings;

extern pros::ADIDigitalOut PTOpiston;
extern bool is_PTO_on_Catapult;

//Initialise sensors
extern pros::Imu inertial;
extern pros::Distance front_dist;
extern pros::Distance back_dist;
extern pros::ADIDigitalIn catapult_switch;
#endif