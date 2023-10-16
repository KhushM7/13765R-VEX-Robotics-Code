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
#include <string>
#include <vector>

double initialCataRotation = 150;

//Autonomous functions
void auton_defensive(){
	robot_move_to(160, "BACK", 1300, true);
	robot_set_heading(270);
	intake.move_velocity(-200);

	pros::delay(500);
	intake.brake();

	//Ram into triball
	//Get away from base of goal
	robot_move_to(100, "BACK", 0, false);
	pros::delay(400);

	//Face triball with back of robot	
	robot_set_heading(90);

	//Ram into it
	robot_move_to(160, "BACK", 0, false);
	pros::delay(1000);

	//Get away from base of goal again
	robot_move_to(-160, "BACK", 0, false);
	pros::delay(125);

	//Go to bar
	robot_set_heading(2);
	robot_move_to(120, "BACK", 80, true);
	robot_set_heading(90);
	robot_move_to(120, "FRONT", 0, false);
	pros::delay(1000);
	stop_robot();
}

void auton_offensive(){
	//Get to goal
	claw.set_value(true);
	robot_move_to(160, "BACK", 1300, true);
	claw.set_value(false);
	robot_set_heading(90);

	//Offload preloaded triball
	intake.move_velocity(-200);
	pros::delay(500);
	intake.brake();

	//Get away from goal to get some space.
	robot_move_to(50, "BACK", 0, false);
	pros::delay(400);	

	//Face preload triball with back of robot
	robot_set_heading(270); 

	//Ram into preload triball
	robot_move_to(160, "BACK", 0, false);
	pros::delay(1000);

	//Get away from the goal
	robot_move_to(-160, "BACK", 0, false);
	pros::delay(125);

	stop_robot();

}

//AUTONOMOUS SELECTOR stuff
typedef enum{
	OFFENSIVE,
	DEFENSIVE,
	SKILLS
} autonStates;
autonStates auton_state = OFFENSIVE;
bool hasConfimed = false;

//LLEMU CALLBACK FUNCTIONS
void onCenterButtonPress(){
	//Set the auton to be defensive
	auton_state = DEFENSIVE;
	pros::lcd::set_text(0, "DEFENSIVE");
}

void onLeftButtonPress(){
	//Set the auton to be offensive
	auton_state = OFFENSIVE;
	pros::lcd::set_text(0, "OFFENSIVE");
}

void onRightButtonPress(){
	//Set the auton to be for skills
	auton_state = SKILLS;
	pros::lcd::set_text(0, "SKILLS   ");
}

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

	
	//Register the callback functions to the buttons
	pros::lcd::register_btn0_cb(onLeftButtonPress);
	pros::lcd::register_btn2_cb(onRightButtonPress);
	pros::lcd::register_btn1_cb(onCenterButtonPress);

	// This loop keeps running until autonomous starts. 
	// Whilst the loop automatically exits upon start of the auton period,
	// we want to be double sure this is the case
	while(!pros::competition::is_autonomous()){
		pros::delay(5);
	}
	
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
	pros::delay(2000);
	auton_offensive();
	
}

//Gets the hottest motor, returning a two character code that represents the motor
// E.g. BR for back right or I  for intake
//After the motor code, it returns the temperature of that motor
void display_hottest_motor(std::vector<pros::Motor> all_motors){
	int highest_motor_temp = 0;
	std::string hottest_motor = "";
	for (int i = 0; i < all_motors.size(); i++){			
		if (all_motors[i].get_temperature() > highest_motor_temp){
			highest_motor_temp = all_motors[i].get_temperature();
			//controller.rumble(".");

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
			else{
				hottest_motor = "C ";
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

        // Define motor labels based on your motor naming convention
        std::string motorLabel = "";
        if (i == 0) {
            motorLabel = "BcLeft: ";
        } else if (i == 1) {
            motorLabel = "BcRight: ";
        } else if (i == 2) {
            motorLabel = "TpLeft: ";
        } else if (i == 3) {
            motorLabel = "TpRight: ";
        } else if (i == 4) {
            motorLabel = "Intake: ";
        } else if (i == 5) {
            motorLabel = "Catapult: ";
        }

        // Display the motor temperature with appropriate color
        pros::lcd::print(i, "%s%.3F", motorLabel.c_str(), all_motors[i].get_temperature());
    }

    // Reset text color to white for any additional text you want to display
    pros::lcd::set_text_color(LV_COLOR_WHITE);
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
	 catapult};

	controller.clear_line(0);

	//Variables for double-binding
	bool wingsAreOpen = false;
	bool isClawDown = false;
	bool isIntakeOff = true;
	

	while(true){
		if (controller.get_analog(ANALOG_LEFT_Y) > 8 || controller.get_analog(ANALOG_LEFT_Y) < -8){
			left_motors.move(controller.get_analog(ANALOG_LEFT_Y));			
		}
		else{
			left_motors.brake();
		}
		
		//These if statements mean the robot will stop even if the controller position is stuck at say 1
		// (prevents robot moving if joystick gets stuck)
		if (controller.get_analog(ANALOG_RIGHT_Y) > 8 || controller.get_analog(ANALOG_RIGHT_Y) < -8){
			right_motors.move(controller.get_analog(ANALOG_RIGHT_Y));
		}
		else{
			right_motors.brake();
		}

		//Turn on intake when Button R1 is held down
		if (controller.get_digital_new_press(DIGITAL_R1)){
			if (isIntakeOff){
				//Forward intake
				intake.move_velocity(600);
				isIntakeOff = false;
			}
			else{
				//Make intake go forward
				intake.brake();
				isIntakeOff = true;
			}
		}
		
		//Turn off intake when Button R2 is pressed
		if (controller.get_digital_new_press(DIGITAL_R2)){
			if (isIntakeOff){
				//Reverse intake
				intake.move_velocity(-600);
				isIntakeOff = false;
			}
			else{
				//Make intake go forward
				intake.brake();
				isIntakeOff = true;
			}
		}

		//Open wings
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

		//*Lift controls*//
		
		//Claw controls
		if (controller.get_digital_new_press(DIGITAL_A)){
			if (isClawDown){
				claw.set_value(false);
				isClawDown = false;
			}
			else{
				claw.set_value(true);
				isClawDown = true;
			}
		}

		if (controller.get_digital(DIGITAL_L2)){
			catapult.move_velocity(60);
		}
		else{
			catapult.brake();
		}


		//Displaying motor temperature stuff
		display_hottest_motor(all_motors);
		display_all_motor_temps(all_motors);

		pros::delay(20);
	}
}
