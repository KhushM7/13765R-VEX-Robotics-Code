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
pros::Motor topLeft(13, MOTOR_GEARSET_18, true);
pros::Motor topRight(11, MOTOR_GEARSET_18, false);

//IF ONLY 4 MOTORS ARE BEING USED CURRENTLY, DELETE THESE TWO LINES OF CODE
pros::Motor midLeft(00, MOTOR_GEARSET_18, false);
pros::Motor midRight(00, MOTOR_GEARSET_18, false);

pros::Motor bottomLeft(10, MOTOR_GEARSET_18, false);
pros::Motor bottomRight(19, MOTOR_GEARSET_18, true);

//IF ONLY 4 MOTORS ARE USED, DELETE midLeft AND midRight FROM THE BELOW MOTOR GROUPS
pros::Motor_Group left_motors({topLeft, bottomLeft, midLeft});
pros::Motor_Group right_motors({topRight, bottomRight, midRight});

pros::Motor intake(8, MOTOR_GEAR_BLUE, false);
pros::Motor catapult1(17, MOTOR_GEAR_RED, true);
pros::Motor catapult2(2, pros::E_MOTOR_GEAR_RED);
pros::Motor flywheel(3, pros::E_MOTOR_GEAR_BLUE);
pros::ADIDigitalOut wings('H');

//Initialise sensors
pros::Imu inertial(18);
pros::Distance front_dist(1);
pros::Distance back_dist(20);
pros::ADIDigitalIn catapult_switch('A');
pros::ADIDigitalIn auton_switch('B');
