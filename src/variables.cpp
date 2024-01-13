#include "variables.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

//Initialise devices
pros::Controller controller(CONTROLLER_MASTER);	

pros::Motor topLeft(19, MOTOR_GEAR_BLUE, true);
pros::Motor topRight(18, MOTOR_GEAR_BLUE, false);

pros::Motor catapultLeft(10, MOTOR_GEAR_GREEN, true);
pros::Motor catapultRight(20, pros::E_MOTOR_GEAR_GREEN, true);
pros::Motor_Group catapult_motors({catapultLeft, catapultRight});

pros::Motor bottomLeft(8, MOTOR_GEAR_BLUE, true);
pros::Motor bottomRight(7, MOTOR_GEARSET_18, false);

pros::Motor_Group left_motors({topLeft, bottomLeft});
pros::Motor_Group right_motors({topRight, bottomRight});

pros::ADIDigitalOut PTOpiston('H');
bool is_PTO_on_Catapult = true;

pros::Motor intake(0, MOTOR_GEAR_BLUE, false);

pros::Motor flywheel(11, pros::E_MOTOR_GEAR_BLUE);
pros::ADIDigitalOut wings('F');

//Initialise sensors
pros::Imu inertial(0);
pros::Distance front_dist(0);
pros::Distance back_dist(0);
pros::ADIDigitalIn catapult_switch('A');
