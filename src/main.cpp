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


//Autonomous functions
void auton_defensive_1(){
	//Jerk intake down by going backward into wall
	robot_set_velocity(-200, 1000);
	robot_set_velocity(100, 400);

	//Move in front of goal
	robot_moveTo_PID("BACK", 1200);

	//Get away from goal a bit
	robot_set_heading_PID(270);
	robot_set_velocity(100, 300);

	//Rotate intake to goal
	robot_set_heading_PID(90);

	//get rid of the triball
	intake.move_velocity(-600);

	//Get away from base of goal
	robot_set_velocity(-100, 400);
	intake.brake();

	//Face triball with back of robot	
	robot_set_heading_PID(270);

	//Ram into it
	robot_set_velocity(-160, 500);

	//Get away from base of goal again
	robot_set_velocity(-150, 700);

	//Go back to matchload bar
	robot_set_heading_PID(359);
	robot_moveTo_PID("BACK", 200);

}

void auton_offensive_1(){
	//Jerk intake down by going backward into wall
	robot_set_velocity(-200, 1000);
	robot_set_velocity(100, 400);

	//Move in front of goal
	robot_moveTo_PID("BACK", 1200);

	//Get away from goal a bit
	robot_set_heading_PID(90);
	robot_set_velocity(100, 300);

	//Rotate intake to goal
	robot_set_heading_PID(270);

	//get rid of the triball
	intake.move_velocity(-600);
	
	//Get away from base of goal
	robot_set_velocity(-100, 400);
	intake.brake();

	//Face triball with back of robot	
	robot_set_heading_PID(90);

	//Ram into it
	robot_set_velocity(-160, 500);

	//Get away from base of goal again
	robot_set_velocity(-150, 700);

	//Go back to matchload bar
	robot_set_heading_PID(359);
	robot_moveTo_PID("BACK", 200);
}

void auton_skills(){
	//Initialise the inertial sensor	
	inertial.reset(true);

	//Start at 45 degree rotation for matchloading on top of matchbar
	//We are facing wrong way due to the catapult being at the back so
	//initial heading is 225 degrees instead.
	inertial.set_heading(225); 

	//We will catapult for 45 seconds roughly
	//Insert catapulting code here
	//Maybe intake corner triball and catapult that as well?

	//After that we will drive forward until we have space to rotate
	//I calculated that we drive forward 860mm roughly.
	robot_moveTo_PID("Front", 860);
	
	//Now rotate to the centre
	robot_set_heading_PID(0);

	//Now move to align with centre of field
	robot_moveTo_PID("BACK", 1270); //Rougly 1270 mm
	
	//Now rotate towards our goal, with our back facing the bar
	robot_set_heading(270);

	//Now open the wings and go forwards the whole way and ram into 
	//triballs on other side to score them
	wings.set_value(true);
	robot_set_velocity(200, 20000);

	//Done!
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

	// This loop keeps running until autonomous starts. 
	// Whilst the loop automatically exits upon start of the auton period,
	// we want to be double sure this is the case
	// while(!pros::competition::is_autonomous()){
	// 	pros::delay(5);
	// }
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
autonStates auton_state = OFFENSIVE;
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
	pros::delay(2000);
	auton_offensive_1();
}

//Gets the hottest motor, printing a two character code that represents the motor
// E.g. BR for back right or I  for intake
//After the motor code, it prints the temperature of that motor
void display_hottest_motor(std::vector<pros::Motor> all_motors){
	int highest_motor_temp = 0;
	std::string hottest_motor = "";
	for (int i = 0; i < all_motors.size(); i++){			
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
	 catapult1,
	 catapult2
	 };

	controller.clear_line(0);

	//Variables for double-binding
	bool wingsAreOpen = false;
	bool isClawDown = false;
	bool isIntakeOff = true;
	

	while(true){
		if (controller.get_digital(DIGITAL_B)){
			auton_defensive_1();
		}
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

		if (controller.get_digital_new_press(DIGITAL_L2)){
			catapult1.move_relative(360, 60);
			catapult2.move_relative(360, 60);
		}

		//Displaying motor temperature stuff
		display_hottest_motor(all_motors);
		display_all_motor_temps(all_motors);

		pros::delay(20);
	}
}
