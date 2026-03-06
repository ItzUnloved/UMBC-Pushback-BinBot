/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

// standard libraries
#include <cstdint>
#include <vector>

// local header files
#include "api.h"
#include "umbc.h"

// used namespaces
using namespace pros;
using namespace umbc;
using namespace std;

// motor constants
#define MOTOR_RED_GEAR_MULTIPLIER       100     // red gearset motor velocity is 100 RPM
#define MOTOR_GREEN_GEAR_MULTIPLIER     200     // green gearset motor velocity is 200 RPM
#define MOTOR_BLUE_GEAR_MULTIPLIER      600     // blue gearset motor velocity is 600 RPM
#define MOTOR_REVERSE                   true    // flag value for reversed motors


// motor ports
#define LEFT_MOTOR_PORTS    {2, -4} // ports for left drive 
#define RIGHT_MOTOR_PORTS   {7, -6} // ports for right motors
 
 
void umbc::Robot::opcontrol() {
 
    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    //left drive
    MotorGroup drive_left(LEFT_MOTOR_PORTS);
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_GREEN);      

    //right drive
    MotorGroup drive_right(RIGHT_MOTOR_PORTS);
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_GREEN);         
 
    // opcontrol loop
    while(true) {   
 
        // getting the values from the controller
        int32_t arcade_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t arcade_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        // calculate velocity for left drive
        int32_t drive_left_velocity = (arcade_y - arcade_x) * MOTOR_GREEN_GEAR_MULTIPLIER / E_CONTROLLER_ANALOG_MAX ;

        // calculate velocity for right drive
        int32_t drive_right_velocity = (arcade_y + arcade_x) * MOTOR_GREEN_GEAR_MULTIPLIER / E_CONTROLLER_ANALOG_MAX;

        // set drive velocity
        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
     }
 }
