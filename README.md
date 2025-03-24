# Art_Robot_ECE4180
Code for the testing and movement of Art Robot

Project uses the MakeBlock 2.0 Robotic kit, specifically 3 encoder motors to faciliate movement.

/***************************************************
 *                Robotics_art_single.ino
 *
 *
 * 2D arm system used to draw images by moving end effector to predetermined locations.
 * Uses Geometric Inverse Kinemetics to determine joint angles as commands are already derivitive based
 * Uses commands such as penUp and penDown to determine if end effector is on page or not
 * Uses PID and PWM to determine voltage to motors.
 * This code takes assumes an initalization position where both arms of the robot are straight outwards.
 ***************************************************/



   /***************************************************
 *                TestEncoders.ino
 *
 * This code tests the encoder readings for the
 * 2D arm system. It prints out the raw encoder 
 * counts and computed angles so you can manually 
 * move the arms and note the limits.
 ***************************************************/
