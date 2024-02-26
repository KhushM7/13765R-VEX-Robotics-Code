//This include guard prevents the header file being included more than once
//This is considered good practice therefore
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <atomic>
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
extern pros::ADIDigitalOut flappy_wings;

extern pros::ADIDigitalOut PTOpiston;
extern std::atomic_bool is_PTO_on_base;

//Initialise sensors
extern pros::Imu inertial;
extern pros::Rotation right_tracker;
extern pros::Rotation back_tracker;
#endif