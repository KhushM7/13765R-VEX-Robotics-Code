//This include guard prevents the header file being included more than once
//This is considered good practice therefore
#include "main.h"
#include "pros/adi.hpp"
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
extern pros::Motor catapult1;
extern pros::Motor catapult2;
extern pros::Motor lift1;
extern pros::Motor lift2;
extern pros::Motor_Group lift;
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut claw;

//Initialise sensors
extern pros::Imu inertial;
extern pros::Distance front_dist;
extern pros::Distance back_dist;
extern pros::Rotation right_tracker;
extern pros::Rotation left_tracker;
#endif