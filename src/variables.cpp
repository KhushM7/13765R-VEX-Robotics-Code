#include "variables.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

//Initialise devices
pros::Controller controller(CONTROLLER_MASTER);	

//PLEASE MODIFY THE PORTS and CHECK ON DEVICES IF THEY NEED TO BE REVERSED OR NOT
//ALL OF THE BASE MOTORS ARE SET TO GREEN. PLEASE MODIFY THIS IF NOT THE CASE
pros::Motor topLeft(1, MOTOR_GEAR_BLUE, true);
pros::Motor topRight(4, MOTOR_GEAR_BLUE, false);

pros::Motor catapultLeft(13, MOTOR_GEAR_GREEN, true);
pros::Motor catapultRight(11, pros::E_MOTOR_GEAR_GREEN, false);
pros::Motor_Group catapult_motors({catapultLeft, catapultRight});

pros::Motor bottomLeft(6, MOTOR_GEAR_BLUE, true);
pros::Motor bottomRight(7, MOTOR_GEARSET_18, false);

pros::Motor_Group left_motors({topLeft, bottomLeft});
pros::Motor_Group right_motors({topRight, bottomRight});
bool is_PTO_on_Catapult = false;

pros::Motor intake(0, MOTOR_GEAR_BLUE, false);

pros::Motor flywheel(0, pros::E_MOTOR_GEAR_BLUE);
pros::ADIDigitalOut wings('H');

//Initialise sensors
pros::Imu inertial(18);
pros::Distance front_dist(0);
pros::Distance back_dist(0);
pros::ADIDigitalIn catapult_switch('A');
pros::ADIDigitalIn auton_switch('B');
