/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

 #include "api.h"
 #include "umbc.h"
 
 #include <cstdint>
 #include <vector>
  
 using namespace pros;
 using namespace umbc;
 using namespace std;
 
 
 #define MOTOR_RED_GEAR_MULTIPLIER       100
 #define MOTOR_GREEN_GEAR_MULTIPLIER     200
 #define MOTOR_BLUE_GEAR_MULTIPLIER      600
 #define MOTOR_REVERSE                   true 
 
 //ports for left motors
 #define LEFT_MOTOR_PORT_1   2
 #define LEFT_MOTOR_PORT_2   -4
 
 //ports for right motors
 #define RIGHT_MOTOR_PORT_1  3
 #define RIGHT_MOTOR_PORT_2  -6
 
 
 void umbc::Robot::opcontrol() {
 
     // nice names for controllers (do not edit)
     umbc::Controller* controller_master = this->controller_master;
     umbc::Controller* controller_partner = this->controller_partner;
 
     // initialize motors and sensors
     std::vector<int8_t> left_motors, right_motors;
 
 
     //left drive
     left_motors.assign({LEFT_MOTOR_PORT_1, LEFT_MOTOR_PORT_2});
 
     MotorGroup drive_left (left_motors);
     drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
     drive_left.set_gearing(E_MOTOR_GEAR_GREEN);      
 
     //right drive
     right_motors.assign({RIGHT_MOTOR_PORT_1, RIGHT_MOTOR_PORT_2});
     
     MotorGroup drive_right(right_motors); 
     drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
     drive_right.set_gearing(E_MOTOR_GEAR_GREEN);         
 
     while(1) {   
 
        //getting the values from the controller
         double arcade_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
         //arcade_y = pow(arcade_y, 3) / (E_CONTROLLER_ANALOG_MAX * E_CONTROLLER_ANALOG_MAX * E_CONTROLLER_ANALOG_MAX);

         double arcade_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
         //arcade_x = pow(arcade_x, 3) / (E_CONTROLLER_ANALOG_MAX * E_CONTROLLER_ANALOG_MAX * E_CONTROLLER_ANALOG_MAX);
 
         double drive_left_velocity = (((double)(arcade_y - arcade_x) / (double)E_CONTROLLER_ANALOG_MAX) * MOTOR_GREEN_GEAR_MULTIPLIER);

         double drive_right_velocity = (((double)(arcade_y + arcade_x) / (double)E_CONTROLLER_ANALOG_MAX) * MOTOR_GREEN_GEAR_MULTIPLIER);
 
         //setting drive velocity
         drive_left.move_velocity(drive_left_velocity);
         drive_right.move_velocity(drive_right_velocity);
 
         // required loop delay (do not edit)
         pros::Task::delay(this->opcontrol_delay_ms);
     }
 }