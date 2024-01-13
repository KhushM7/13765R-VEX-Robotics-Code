#include "main.h"
#include "autonomous_functions.h"
#include "display/lv_misc/lv_color.h"
#include "pros/colors.h"
#include "pros/llemu.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "variables.h"
#include <cmath>
#include <string>
#include <vector>
#include <atomic>

//PTO variable
std::atomic_bool isSwitchingPTO = false;

//Autonomous functions
void auton_defensive_1(){}


void auton_offensive_1(){}

void auton_skills(){}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	while (!pros::lcd::is_initialized()){
		pros::delay(20);
	}
	pros::lcd::set_background_color(LV_COLOR_BLACK);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

//AUTONOMOUS SELECTOR stuff
typedef enum{
	OFFENSIVE,
	DEFENSIVE,
	SKILLS
} autonStates;
autonStates auton_state = SKILLS; //Default. 
//Also may not be able to select auton during skills
bool hasConfimed = false;

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	while (!pros::competition::is_autonomous()){
		if(pros::lcd::read_buttons() & LCD_BTN_LEFT){
			//Set the auton to be offensive
			auton_state = OFFENSIVE;
			pros::lcd::set_text(0, "OFFENSIVE");
		}
		if (pros::lcd::read_buttons() & LCD_BTN_CENTER){
			//Set the auton to be defensive
			auton_state = DEFENSIVE;
			pros::lcd::set_text(0, "DEFENSIVE");			
		}
		if (pros::lcd::read_buttons() & LCD_BTN_RIGHT){
			//Set the auton to be for skills
			auton_state = SKILLS;
			pros::lcd::set_text(0, "SKILLS   ");	
		}
		pros::delay(5);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	auton_defensive_1();
}

//Gets the hottest motor, printing a two character code that represents the motor
// E.g. BR for back right or I  for intake
//After the motor code, it prints the temperature of that motor
void display_hottest_motor(std::vector<pros::Motor> all_motors){
	int highest_motor_temp = all_motors[0].get_temperature();
	std::string hottest_motor = "";
	for (int i = 1; i < all_motors.size(); i++){			
		if (all_motors[i].get_temperature() > highest_motor_temp){
			highest_motor_temp = all_motors[i].get_temperature();
			//Using order of motors when list was created
			if (i == 0){
				hottest_motor = "BR";				
			}
			else if (i == 1){
				hottest_motor = "BL";
			}
			else if (i == 2){
				hottest_motor = "TL";
			}
			else if (i == 3){
				hottest_motor = "TR";
			}
			else if (i == 4){
				hottest_motor = "I ";
			}
			else if (i == 5){
				hottest_motor = "C1";
			}
			else{
				hottest_motor = "C2";
			}
		}
	}
	controller.print(0,0,"%s %.3F", hottest_motor, highest_motor_temp);
}

// Display all motor temperatures on brain screen
void display_all_motor_temps(std::vector<pros::Motor> all_motors) {
    // Display base motor temperatures
    for (int i = 0; i < all_motors.size(); i++) {
        // Check if the motor temperature is above 40 degrees Celsius
        if (all_motors[i].get_temperature() > 40) {
            pros::lcd::set_text_color(LV_COLOR_RED); // Set text color to red
        } else {
            pros::lcd::set_text_color(LV_COLOR_WHITE); // Set text color to white for other motors
        }

        // Define motor labels based on original motor naming convention
        std::string motorLabel = "";
        if (i == 0) {
            motorLabel = "BcLeft:  ";
        } else if (i == 1) {
            motorLabel = "BcRight: ";
        } else if (i == 2) {
            motorLabel = "TpLeft:  ";
        } else if (i == 3) {
            motorLabel = "TpRight: ";
        } else if (i == 4) {
            motorLabel = "Intake:  ";
        } else if (i == 5) {
            motorLabel = "Cata-1:  ";
        } else if (i == 6){
			motorLabel = "Cata-2:  ";
		} else if (i == 7){
			motorLabel = "FlyWheel:";
		}
		
		

        // Display the motor temperature with appropriate color
        pros::lcd::print(i, "%s%.3F", motorLabel.c_str(), all_motors[i].get_temperature());
    }

    // Reset text color to white for any additional text you want to display
    pros::lcd::set_text_color(LV_COLOR_WHITE);
}

void display_PTO_state(){
	if (!is_PTO_on_base.load()){
		controller.print(0, 0, "4-motor drive");
	}
	else{
		controller.print(0, 0, "6-motor drive");
	}
}

void switch_PTO_state(){
	while (true){
		if (isSwitchingPTO.load()){
			//Set the base motors to rotate forward
			left_motors.move_velocity(200);
			right_motors.move_velocity(200);

			//Wait for about 0.75s
			pros::delay(750);

			//Extend pistons in the correct way
			if (!is_PTO_on_base.load()){
				//Switch to 6 motor drive
				PTOpiston.set_value(1);
			}
			else{
				//Switch to 4 motor drive
				PTOpiston.set_value(0);
			}	
		}
		pros::delay(40);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	std::vector<pros::Motor> all_motors = 
	{bottomLeft,
	 bottomRight,
	 topLeft,
	 topRight,
	 intake,
	 catapultLeft,
	 catapultRight,
	 flywheel
	 };

	controller.clear_line(0);

	//Variables for double-binding
	bool wingsAreOpen = false;
	bool isClawDown = false;
	bool isIntakeOff = true;
	bool hangIsDown = false;
	bool catapultIsMoving = false;

	//To ensure catapult cannot be touched whilst its shooting
	bool flywheelIsMoving = false;
	bool hasLeftBumperSwitch = false;
	

	//Start display task which will display both motor temperatures
	// and the current PTO state
	pros::Task display_task([=](){
		while(true){
			display_all_motor_temps(all_motors);
			display_PTO_state();
			pros::delay(500);
		}
	});

	pros::Task PTO_switching_task(switch_PTO_state);

	while(true){
		//Drive bas code
		if (!isSwitchingPTO.load()){			
			if (controller.get_analog(ANALOG_LEFT_Y) > 8 || controller.get_analog(ANALOG_LEFT_Y) < -8){
				left_motors.move(controller.get_analog(ANALOG_LEFT_Y));

				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultLeft.move(controller.get_analog(ANALOG_LEFT_Y));
				}			
			}
			
			else
			{
				left_motors.brake();

				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultLeft.move(controller.get_analog(ANALOG_LEFT_Y));
				}	
			}
		
			//These if statements mean the robot will stop even if the controller position is stuck at say 1
			// (prevents robot moving if joystick gets stuck)
			if (controller.get_analog(ANALOG_RIGHT_Y) > 8 || controller.get_analog(ANALOG_RIGHT_Y) < -8){
				right_motors.move(controller.get_analog(ANALOG_RIGHT_Y));
				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultRight.move(controller.get_analog(ANALOG_LEFT_Y));
				}
			}
			else{
				right_motors.brake();
				//If we are on 6 motor drive...
				if (is_PTO_on_base.load()){
					catapultRight.brake();
				}
			}
		}


		//Toggle intake when Button R1 is pressed (make intake go forward)
		if (controller.get_digital_new_press(DIGITAL_R1)){
			if (isIntakeOff){
				//Forward intake
				intake.move_velocity(600);
				isIntakeOff = false;
			}
			else{
				//Make intake stop
				intake.brake();
				isIntakeOff = true;
			}
		}

		
		
		//Turn off intake when Button R2 is pressed (make intake reverse)
		if (controller.get_digital_new_press(DIGITAL_R2)){
			if (isIntakeOff){
				//Reverse intake
				intake.move_velocity(-600);
				isIntakeOff = false;
			}
			else{
				//Make intake stop
				intake.brake();
				isIntakeOff = true;
			}
		}

		//Wings
		if (controller.get_digital_new_press(DIGITAL_L1)){
			if (wingsAreOpen){
				wings.set_value(false);
				wingsAreOpen = false;
			}
			else{
				wings.set_value(true);
				wingsAreOpen = true;
			}
		}

		//Flywheel
		if (controller.get_digital_new_press(DIGITAL_A)){
			if (flywheelIsMoving){
				//Stop flywheel
				flywheel.brake();
				flywheelIsMoving = false;
			}
			else{
				//Make flywheel spin at full
				flywheel.move_voltage(12000);
				flywheelIsMoving = true;
			}
		}

		if (controller.get_digital_new_press(DIGITAL_Y)){
			if (flywheelIsMoving){
				//Stop flywheel
				flywheel.brake();
				flywheelIsMoving = false;
			}
			else{
				//Make flywheel spin at reverse
				flywheel.move_velocity(-600);
				flywheelIsMoving = true;
			}
		}  

		//Lift code
		if (controller.get_digital_new_press(DIGITAL_L2)){
			if (is_PTO_on_base.load()){
				isSwitchingPTO = true;
			}
			else{
				//Run lift code
				//For now it is just a vibration on controller
				controller.rumble("-");
			}
		}

		//Catapult code (commented out)
		//The catapult will try and go up if the PTO is in position
		//Catapult stuff
		/*if (controller.get_digital_new_press(DIGITAL_DOWN) && is_PTO_on_base.load()){
			catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			catapult_motors.move_relative(-360, -100);
			catapultIsMoving = false;
			hasLeftBumperSwitch = false;
		}

		if (controller.get_digital_new_press(DIGITAL_L2) && !catapultIsMoving){
			catapultIsMoving = true;

			//Switch PTO if we are using 6 motor drive
			if (!is_PTO_on_base.load()){
				PTOpiston.set_value(0);				
			}

			//Change brake mode to reduce strain on motors
			catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
			catapult_motors.brake();
			
			hasLeftBumperSwitch = false;	
		}

		if (catapultIsMoving){
			if (!hasLeftBumperSwitch){
				catapult_motors.move_velocity(100);
				if (catapult_switch.get_value() == 0){
					hasLeftBumperSwitch = true;
				}
			}
			//Keep rotating catapult until
			else if (catapult_switch.get_value() == 0){
				catapult_motors.move_velocity(60);
			}
			//Once we hit the bumper switch			
			else{
				catapult_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
				catapult_motors.brake();
				catapultIsMoving = false;
			}
		}*/

		//Switch to 6 motor drive
		if (controller.get_analog(ANALOG_LEFT_X) <= -120
		&& controller.get_analog(ANALOG_LEFT_Y) <= -120)
		{
			if (is_PTO_on_base.load()){
				isSwitchingPTO.store(true);
			}
		}	

		

		pros::delay(10); //Refresh rate of a motor
	}
}
