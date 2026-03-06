#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int BALLGOAL_SPEED = 50;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I //26,330
  chassis.pid_drive_constants_set(9,0,31.5);     // Fwd/rev constants, used for odom and non odom motions - 26,0,245 old bot and 25,0,260 and 18,0,125
  chassis.pid_heading_constants_set(2.5,0,16.5); //ds the robot straight while going forward without odom - 11,0,20
  chassis.pid_turn_constants_set(2.5, 0, 16.5);     // Turn in place constants - 2.5,0,16.5 old bot
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void thirdPart(){
  chassis.drive_angle_set(45_deg); //face forward
  chassis.pid_wait();
  chassis.pid_drive_set(54_in, 20); //30
  chassis.pid_wait_until(6_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(DRIVE_SPEED+5+5);
  chassis.pid_wait_until(30_in); //wait until 30 inches to start slowing down
  lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  middleGoalScore.set(false);

  chassis.pid_wait();

  intake.move(0);
  middleIntakeMotor.move(127);
  //lilRaaahh.set(true);

  chassis.pid_turn_set(0_deg, TURN_SPEED+10); //face the matchloader 
  chassis.pid_wait();
  pros::delay(100);
  // lilRaaahh.set(true);
  middleIntakeMotor.move(127); //run bottom intake motor to intake the 3 balls
  topIntakeMotor.move(-15);
  //topIntakeMotor.move(0); //stop top intake motor
  chassis.pid_drive_set(29_in, BALLGOAL_SPEED); //30
  chassis.pid_wait_until(5_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //backward to goal 15 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract
  chassis.pid_turn_set(315_deg,TURN_SPEED); // 135 deg to perimeter wall
  chassis.pid_wait_until(325_deg); //wait until 140 deg before switching to drive speed +10
  pros::delay(100);
  intake.move(0); //intake stop spinning

  chassis.pid_drive_set(-17_in,DRIVE_SPEED+5); //22 in towards perimeter wall
  chassis.pid_wait_quick();
  chassis.pid_turn_set(2_deg,TURN_SPEED); //turn towards other side matchload/perimeter 180 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-66_in,DRIVE_SPEED-5); //go towards other side 65 in 60 -10-5
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg,TURN_SPEED+10); //turn 130 before now 230 deg to start alinging with long goal
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-10_in,DRIVE_SPEED+5); //16 inches diagonal movement to long goal
  chassis.pid_wait_quick();
  middleIntakeMotor.move(-10);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 180 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-13_in,DRIVE_SPEED+10); //backward to long goal by 15
  chassis.pid_wait_until(-10_in); //wait until -10 inches before stopping
  middleIntakeMotor.move(127); //intaking at goals with bottom stage
  topIntakeMotor.move(127); //top stage intake spins passive back to prevent jamming
  pros::delay(150);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards long goal 180
  intake.move(127); //intaking at goals
  pros::delay(1900); //delay for scoring at long goal 
  intake.move(0);
}

void sevenMid(){
  topIntakeMotor.move(-127); //top stage intake spins passive back to score
  middleIntakeMotor.move(-25);//bottom stage intake spins to score
  pros::delay(2000); //time for intakes to spin up
  middleGoalScore.set(true); //deploy middle goal scoring piston
  pros::delay(200); //delay for piston to deploy before outtaking balls
  middleIntakeMotor.move(90); //bottom stage intake spins to score
  topIntakeMotor.move(-10); //top stage intake spins passive back to score
  chassis.pid_drive_set(-15_in,35); //towards goal 30 inches
  //chassis.pid_wait();
  pros::delay(1000); //3500
  middleIntakeMotor.move(80); //bottom stage intake spins to pick up fallen balls
  pros::delay(500); //time for intaking fallen balls
  middleIntakeMotor.move(75); //bottom stage intake spins to pick up fallen balls
  pros::delay(1500); //time for intaking fallen balls
  //chassis.pid_drive_set(0.5_in,50); //towards goal 30 inches
  //pros::delay(1000); //3500
  middleIntakeMotor.move(70); //bottom stage intake spins to pick up fallen balls
  pros::delay(1500); //time for intaking fallen balls
  chassis.pid_drive_set(3_in,50); //towards goal 30 inches
  chassis.pid_wait();
  //pros::delay(2000); //3500
  middleGoalScore.set(false); //retract middle goal scoring piston
  chassis.pid_drive_set(-2_in, BALLGOAL_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  middleGoalScore.set(true); //deploy middle goal scoring piston to park in the zone
  chassis.pid_drive_set(6_in, 15); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
}

void finalSkills(){
  // chassis.drive_angle_set(0_deg); //face forward
  // chassis.pid_wait();
  // descore.set(true); //make sure descore is retracted at beginning of match
  // middleIntakeMotor.move(127); //bottom stage intake spins 
  // topIntakeMotor.move(-100); //top stage intake spins passive back 
  // chassis.pid_turn_set(332_deg,TURN_SPEED); //turn towards matchloader 335
  // chassis.pid_wait();
  // chassis.pid_drive_set(26_in, DRIVE_SPEED+5); //drive 32 inches toward 3 cluster of balls at drive_speed+5
  // chassis.pid_wait_until(13_in); //after 14 inches switch to ballgoalspeed
  // lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls

  // //chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // chassis.pid_wait_quick_chain();
  // lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  // chassis.pid_turn_set(238_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  // chassis.pid_wait_quick();
  // topIntakeMotor.move(-127); //pasive top stage intake to prevent jamming
  // middleIntakeMotor.move(-15); //outake intake

  // chassis.pid_drive_set(-15_in,DRIVE_SPEED); //drive backwards into goal by 15 inches
  // chassis.pid_wait_until(-11_in); //wait until -2 inches 
  // middleGoalScore.set(true); //deploy middle goal scoring piston 
  // //chassis.pid_wait();
  // pros::delay(200); //delay for piston to deploy before outtaking balls

  // middleIntakeMotor.move(100);  //socre on mid goal 100
  // topIntakeMotor.move(-60); //run top stage intake forward at reduced speed initally out
  // chassis.pid_turn_set(226_deg,TURN_SPEED); //turn towards matchloader 335
  // pros::delay(300);
  // chassis.pid_drive_set(-2_in, 40); //drive 15 inches toward 3 cluster of balls at drive_speed
  // pros::delay(700); //time for middle goal scoring - 1600 ms 
  // middleGoalScore.set(false); //retract middle goal scoring piston 
  // //intake.move(127); // get rid of extra balls in the intake 

  chassis.drive_angle_set(0_deg); //face forward
  chassis.pid_wait();
  descore.set(true); //make sure descore is retracted at beginning of match
  middleIntakeMotor.move(127); //bottom stage intake spins 
  topIntakeMotor.move(-100); //top stage intake spins passive back 
  chassis.pid_turn_set(334.5_deg,TURN_SPEED); //turn towards matchloader 335
  chassis.pid_wait();
  chassis.pid_drive_set(29_in, DRIVE_SPEED+5); //drive 32 inches toward 3 cluster of balls at drive_speed+5
  chassis.pid_wait_until(20_in); //after 14 inches switch to ballgoalspeed
  lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls

  //chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(238_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait_quick();
  lilRaaahh.set(false); //deploy matchloader mechanism to pick up balls
  // lilRaaahh.set(false); //deploy matchloader mechanism to pick up balls
  topIntakeMotor.move(-127); //pasive top stage intake to prevent jamming
  middleIntakeMotor.move(-15); //outake intake

  chassis.pid_drive_set(-15_in,DRIVE_SPEED); //drive backwards into goal by 15 inches
  chassis.pid_wait_until(-11_in); //wait until -2 inches 
  middleGoalScore.set(true); //deploy middle goal scoring piston 
  //chassis.pid_wait();
  pros::delay(200); //delay for piston to deploy before outtaking balls

  middleIntakeMotor.move(100);  //socre on mid goal 100
  topIntakeMotor.move(-60); //run top stage intake forward at reduced speed initally out
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn towards matchloader 335
  pros::delay(300);
  chassis.pid_drive_set(-2_in, 40); //drive 15 inches toward 3 cluster of balls at drive_speed
  pros::delay(700); //time for middle goal scoring - 1600 ms 
  middleGoalScore.set(false); //retract middle goal scoring piston 
  //intake.move(127); // get rid of extra balls in the intake 

  chassis.pid_drive_set(53_in, DRIVE_SPEED-5-5); //drive 54 inches towards field perimeter wall
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader 
  chassis.pid_wait();


  middleIntakeMotor.move(127); //running the bottom intake stage to matchload
  topIntakeMotor.move(-30); //pasive top stage intake to prevent jamming
  chassis.pid_drive_set(19_in, DRIVE_SPEED+10); //drive in by 19 incehs into matchloader 
  chassis.pid_wait_until(2_in); //wait for 2 inches before switching to ballgoal speed -5
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5); 
  chassis.pid_wait();
  middleGoalDescore.set(true); //deploy middle goal descore piston to descore the middle goal if needed
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();

  pros::delay(700); //time for matchloading 
  middleGoalDescore.set(false); //retract middle goal descore piston
  chassis.pid_drive_set(-17_in, DRIVE_SPEED+10); //backward to goal 17 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract

  // //turning to other side/going to other side
  chassis.pid_turn_set(120_deg,TURN_SPEED+10); //300 deg to perimeter wall 300 before 
  // pros::delay(1000); //1000
  chassis.pid_wait_until(130_deg); //wait until 140 deg before switching to drive speed +10
  pros::delay(100);
  intake.move(0); //intake stop spinning
  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //16 in towards perimeter wall 14 before 
  chassis.pid_wait_quick();
  chassis.pid_turn_set(182_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 0 deg before 
  //pros::delay(300);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-66_in,DRIVE_SPEED-5); //go towards other side 65 in 64 -10-5-5
  chassis.pid_wait();

  // //next quadrant
  // chassis.pid_turn_set(53_deg,TURN_SPEED+10); //turn 50 deg to start alinging with long goal
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(15.5_in,DRIVE_SPEED+10); //17 inches diagonal movement to long goal 15.5
  // // pros::delay(650);
  // chassis.pid_wait();  
  // chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards matchload with bot facing forwards 0 deg
  // // chassis.pid_wait_quick_chain(); 
  // pros::delay(300);
  // chassis.pid_drive_set(-17_in,DRIVE_SPEED+10); //backward to long goal by 15
  // // chassis.pid_wait();
  // pros::delay(400);

  chassis.pid_turn_set(270_deg,TURN_SPEED+10); //turn 50 deg to start alinging with long goal
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED); //drive to collect balls fallen balls  -40 ((frontDistance.get())*0.0393701) -29.5
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards matchload with bot facing forwards 0 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-9_in,DRIVE_SPEED+10); //backward to long goal by 15
  chassis.pid_wait_until(-6_in); //wait until -10 inches before stopping
  middleIntakeMotor.move(127); //intaking at goals with bottom stage
  topIntakeMotor.move(127); //top stage intake spins passive back to prevent jamming
  pros::delay(150);

  //intake.move(127); //intaking at goals

  chassis.pid_turn_set(0_deg,TURN_SPEED); //turn towards long goal 180

  //middleIntakeMotor.move(110); //bottom stage intake spins
  //topIntakeMotor.move(110); //top stage intake spins passive back to prevent j
  pros::delay(1950); //delay for scoring at long goal 
  intake.move(0);
  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-15);
  pros::delay(100);
  chassis.pid_drive_set(39_in,DRIVE_SPEED); //forward 43
  chassis.pid_wait_until(12_in); //at 12 inches switch to ballgoalspeed+10
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(600); //delay for intaking at matchload 600 ms
  intake.move(-5); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-31_in,DRIVE_SPEED+10); //backwards 33 
  chassis.pid_wait();
  //intake.move(100); //intake forward to score
  middleIntakeMotor.move(127); //bottom stage intake spins to score
  topIntakeMotor.move(127); //pasive top stage intake to prevent jamming
  //middleIntakeMotor.move(110);
  //topIntakeMotor.move(80);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127);//back forth goals for control 4 out 
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 20); //back forth goals for control 5 in
  chassis.pid_wait();
  lilRaaahh.set(false); //retract matchloader mechanism

  middleIntakeMotor.move(127); //bottom stage intake spins
  topIntakeMotor.move(-20); //top stage intake spins passive back
  chassis.pid_drive_set(21_in,DRIVE_SPEED); //21
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED); //40
  chassis.pid_wait();
  chassis.pid_drive_set(30_in,DRIVE_SPEED); //33 +10 and 42
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 65); //drive forward 60 inches into the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED); //turn towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(((frontDistance.get())*0.0393701) -29.5, DRIVE_SPEED); //drive to collect balls fallen balls  -40
  chassis.pid_wait();
  chassis.pid_turn_set(50_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-46_in, DRIVE_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(315_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  chassis.pid_wait();
  topIntakeMotor.move(-127); //top stage intake spins passive back to score
  middleIntakeMotor.move(-25); //bottom stage intake spins to pick up fallen balls
  //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls

  //intake.move(-30); //intake spins to pick up fallen balls
  chassis.pid_drive_set(-9_in, DRIVE_SPEED-20); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait_until(-8_in); //wait until -10 inches before stopping
  middleGoalScore.set(true); //deploy middle goal scoring piston

  chassis.pid_wait();
  //middleGoalScore.set(true);
  middleIntakeMotor.move(90); //bottom stage intake spins to score
  topIntakeMotor.move(0); //top stage intake spins passive back to score
  chassis.pid_drive_set(-15_in,50); //towards goal 30 inches
  //chassis.pid_wait();
  pros::delay(500); //3500
  chassis.pid_drive_set(0.5_in,50); //towards goal 30 inches
  pros::delay(500); //3500
  middleIntakeMotor.move(90); //bottom stage intake spins to pick up fallen balls
  pros::delay(1800); //time for intaking fallen balls
  middleIntakeMotor.move(65); //bottom stage intake spins to pick up fallen balls
  chassis.pid_drive_set(-2_in, BALLGOAL_SPEED); //drive back 20 inches to be fully in the parking zone
  //chassis.pid_drive_set(1_in,50); //towards goal 30 inches
  //chassis.pid_wait();
  // pros::delay(1000); //3500
  // chassis.pid_drive_set(-1_in,50); //towards goal 30 inches
  // chassis.pid_wait();
  // pros::delay(1000); //3500
  // chassis.pid_drive_set(6_in,20); //towards goal 30 inches

  // chassis.pid_drive_set(54_in,DRIVE_SPEED+5+5); //move toward field perimeter by 53 inches 
  // chassis.pid_wait();
  // middleGoalScore.set(false);

  chassis.pid_drive_set(54_in, 20); //30
  chassis.pid_wait_until(6_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(DRIVE_SPEED+5+5);
  chassis.pid_wait_until(30_in); //wait until 30 inches to start slowing down
  lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  middleGoalScore.set(false);

  chassis.pid_wait();

  intake.move(0);
  middleIntakeMotor.move(127);
  //lilRaaahh.set(true);

  chassis.pid_turn_set(0_deg, TURN_SPEED+10); //face the matchloader 
  chassis.pid_wait();
  pros::delay(100);
  // lilRaaahh.set(true);
  middleIntakeMotor.move(127); //run bottom intake motor to intake the 3 balls
  topIntakeMotor.move(-15);
  //topIntakeMotor.move(0); //stop top intake motor
  chassis.pid_drive_set(29_in, BALLGOAL_SPEED); //30
  chassis.pid_wait_until(5_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //backward to goal 15 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract
  chassis.pid_turn_set(135_deg,TURN_SPEED); // 135 deg to perimeter wall
  chassis.pid_wait();
  intake.move(0); //intake stop spinning

  chassis.pid_drive_set(17_in,DRIVE_SPEED+5); //22 in towards perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards other side matchload/perimeter 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(54_in,DRIVE_SPEED-5); //go towards other side 65 in 60 -10-5
  chassis.pid_wait();
  chassis.pid_turn_set(230_deg,TURN_SPEED+10); //turn 130 before now 230 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(18_in,DRIVE_SPEED+5); //16 inches diagonal movement to long goal
  chassis.pid_wait();
  middleIntakeMotor.move(-10);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in,DRIVE_SPEED+5); //backward to long goal by 22 in skillsCut
  // chassis.pid_wait();
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1900); //delay for scoring at long goal 
  intake.move(0);

  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-30);
  pros::delay(100);
  chassis.pid_drive_set(41_in,DRIVE_SPEED); //forward 47 in skillsCut towards matchloader 
  chassis.pid_wait_until(15_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5); //+2
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(450); //delay for intaking at matchload 450 ms

  //intake.move(-10); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-32_in,DRIVE_SPEED+5+5); //backwards 33 before in skillsCut/now 37 to long goal 
  pros::delay(500);
  middleIntakeMotor.move(-10);
  // // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // // topIntakeMotor.move(100);
  //intake.move(127);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(127);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127); //back forth goals for control 4 out 
  pros::delay(450);
  chassis.pid_drive_set(-5_in, 80); //back forth goals for control 5 in
  pros::delay(450);
  // chassis.pid_drive_set(4_in,DRIVE_SPEED); //back forth goals for control 4 out 
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  // chassis.pid_wait();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //new park

  chassis.drive_angle_set(180_deg);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(235_deg, TURN_SPEED); //228
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(37_in,DRIVE_SPEED+10); //33 +10 and 42
  //chassis.pid_wait();
  pros::delay(1100);
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120); //265
  //chassis.pid_wait();
  pros::delay(600);
  // chassis.pid_drive_set(35_in, 125); //125 and 35
  // chassis.pid_wait();
  chassis.pid_drive_set(29_in, 70 ); //125 and 28
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4_in, 85); //125 and 35
  chassis.pid_wait_quick_chain();
}

void newMiddle(){
  middleIntakeMotor.move(127); //bottom stage intake spins
  topIntakeMotor.move(-20); //top stage intake spins passive back
  chassis.pid_drive_set(21_in,DRIVE_SPEED); //21
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED); //40
  chassis.pid_wait();
  chassis.pid_drive_set(30_in,DRIVE_SPEED); //33 +10 and 42
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 65); //drive forward 60 inches into the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED); //turn towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(((frontDistance.get())*0.0393701) -31, DRIVE_SPEED); //drive to collect balls fallen balls  -40
  chassis.pid_wait();
  chassis.pid_turn_set(50_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-46_in, DRIVE_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(315_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  chassis.pid_wait();
  topIntakeMotor.move(-127); //top stage intake spins passive back to score
  middleIntakeMotor.move(-25); //bottom stage intake spins to pick up fallen balls
  //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls

  //intake.move(-30); //intake spins to pick up fallen balls
  chassis.pid_drive_set(-9_in, DRIVE_SPEED-20); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait_until(-8_in); //wait until -10 inches before stopping
  middleGoalScore.set(true); //deploy middle goal scoring piston

  chassis.pid_wait();
  //middleGoalScore.set(true);
  middleIntakeMotor.move(90); //bottom stage intake spins to score
  topIntakeMotor.move(0); //top stage intake spins passive back to score
  chassis.pid_drive_set(-15_in,50); //towards goal 30 inches
  //chassis.pid_wait();
  pros::delay(500); //3500
  chassis.pid_drive_set(0.5_in,50); //towards goal 30 inches
  pros::delay(500); //3500
  middleIntakeMotor.move(90); //bottom stage intake spins to pick up fallen balls
  pros::delay(1800); //time for intaking fallen balls
  middleIntakeMotor.move(70); //bottom stage intake spins to pick up fallen balls
  chassis.pid_drive_set(-2_in, BALLGOAL_SPEED); //drive back 20 inches to be fully in the parking zone
  //chassis.pid_drive_set(1_in,50); //towards goal 30 inches
  //chassis.pid_wait();
  // pros::delay(1000); //3500
  // chassis.pid_drive_set(-1_in,50); //towards goal 30 inches
  // chassis.pid_wait();
  // pros::delay(1000); //3500
  chassis.pid_drive_set(6_in,20); //towards goal 30 inches

  // middleIntakeMotor.move(127);
  // topIntakeMotor.move(-10);
  // chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  // chassis.pid_wait();
  // middleIntakeMotor.move(-20);
  // topIntakeMotor.move(-127);
  // chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  // chassis.pid_wait();
  // lilRaaahh.set(true);
  // //intake.move(-40);
  // chassis.pid_drive_set(-17_in, DRIVE_SPEED); //drive back 23
  // chassis.pid_wait_until(-15_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // middleGoalScore.set(true);
  // chassis.pid_wait();
  // middleIntakeMotor.move(110); //85 bottom 
  // topIntakeMotor.move(-127); //-70 top 
  // chassis.pid_drive_set(-20_in,50); //towards goal 30 inches
  // chassis.pid_wait();
  // pros::delay(3000); //3500
  // lilRaaahh.set(false);
}

void matchload(int matchloadNumber){
  optical_sensor.set_led_pwm(100); //sets led brightness to 100%
  pros::delay(15); //15 ms delay to allow led to turn on

  double blueVal = 0.0; //value of blue hue
  double blueMin = 0.0; //min blue hue
  double blueMax = 0.0; //max blue hue

  double redVal = 0.0; //value of red hue 
  double redMin = 0.0; //mix red hue
  double redMax = 0.0; //max red hue

  double distanceMax = 0.0; //distance to ball

  uint32_t lastDetectTime = 0; //last time a ball was detected
  uint32_t firstDetectTime = 0; //first time a ball was detected

  while (true){ //while true
    double currValue = optical_sensor.get_hue(); //gets current hue value
    double currDistance = optical_sensor.get_proximity(); //gets current distance value

    bool ballDetected = ((blueMin <= currValue && currValue <= blueMax) || (redMin <= currValue && currValue <= redMax)) && (currDistance <= distanceMax); //checks if ball is detected in color range and within distance
    //bool ballDetected = (blueMin <= currValue && currValue <= blueMax) || (redMin <= currValue && currValue <= redMax); //checks if ball is detected and in red or blue hue range 

    if (ballDetected){
      if (firstDetectTime == 0) {
        firstDetectTime = pros::millis(); //set first detect time
      }
      lastDetectTime = pros::millis(); //updates last detect time
      if (matchloadNumber == 1 || matchloadNumber == 4){
        chassis.pid_turn_set(178_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
        chassis.pid_turn_set(182_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
      }
      if (matchloadNumber == 2 || matchloadNumber == 3){
        chassis.pid_turn_set(358_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
        chassis.pid_turn_set(2_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
      }
    }

    if (pros::millis() - lastDetectTime > 500 || (firstDetectTime > 0 && pros::millis() - firstDetectTime > 5000)){ //if more than 500 ms has passed since last detection or 5000 ms since first
      if (matchloadNumber == 1 || matchloadNumber == 4){
        chassis.pid_turn_set(180_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
      }
      if (matchloadNumber == 2 || matchloadNumber == 3){
        chassis.pid_turn_set(360_deg, TURN_SPEED); //turn 
        chassis.pid_wait();
      }

      optical_sensor.set_led_pwm(0); //sets led brightness to 0%

      break; //exit loop
    }

    pros::delay(50); //50 ms delay

  }
}

void calibrateRed(int matchloaderNumber){
  std::vector<double> redHues;
  double redHueSum = 0.0;
  double redMin = 0.0;
  double redMax = 0.0;
  double redAvg = 0.0;

  optical_sensor.set_led_pwm(100); //sets led brightness to 100%

  for (int i = 0; i < 20; i++){  // 20 iterations for data collection
    if (matchloaderNumber == 1 || matchloaderNumber == 4){
      chassis.pid_turn_set(178_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      double hue = optical_sensor.get_hue();
      redHues.push_back(hue);
      redHueSum += hue;
      chassis.pid_turn_set(182_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      hue = optical_sensor.get_hue();
      redHues.push_back(hue);
      redHueSum += hue;
    }
    if (matchloaderNumber == 2 || matchloaderNumber == 3){
      chassis.pid_turn_set(358_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      double hue = optical_sensor.get_hue();
      redHues.push_back(hue);
      redHueSum += hue;
      chassis.pid_turn_set(2_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      hue = optical_sensor.get_hue();
      redHues.push_back(hue);
      redHueSum += hue;
    }
  }

  if (!redHues.empty()) {
    redAvg = redHueSum / redHues.size();
    redMin = *std::min_element(redHues.begin(), redHues.end());
    redMax = *std::max_element(redHues.begin(), redHues.end());
  }

  optical_sensor.set_led_pwm(0); //sets led brightness to 0%

  pros::delay(50); //delay calibration

  // Store or use redMin and redMax as needed
  printf("Red Avg: %.2f\n", redAvg);
  printf("Red Min: %.2f\n", redMin);
  printf("Red Max: %.2f\n", redMax);
}

void calibrateBlue(int matchloaderNumber){
  std::vector<double> blueHues;
  double blueHueSum = 0.0;
  double blueMin = 0.0;
  double blueMax = 0.0;
  double blueAvg = 0.0;

  optical_sensor.set_led_pwm(100); //sets led brightness to 100%

  for (int i = 0; i < 20; i++){  // 20 iterations for data collection
    if (matchloaderNumber == 1 || matchloaderNumber == 4){
      chassis.pid_turn_set(178_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      double hue = optical_sensor.get_hue();
      blueHues.push_back(hue);
      blueHueSum += hue;
      chassis.pid_turn_set(182_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      hue = optical_sensor.get_hue();
      blueHues.push_back(hue);
      blueHueSum += hue;
    }
    if (matchloaderNumber == 2 || matchloaderNumber == 3){
      chassis.pid_turn_set(358_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      double hue = optical_sensor.get_hue();
      blueHues.push_back(hue);
      blueHueSum += hue;
      chassis.pid_turn_set(2_deg, TURN_SPEED); //turn 
      chassis.pid_wait();
      hue = optical_sensor.get_hue();
      blueHues.push_back(hue);
      blueHueSum += hue;
    }
  }

  if (!blueHues.empty()) {
    blueAvg = blueHueSum / blueHues.size();
    blueMin = *std::min_element(blueHues.begin(), blueHues.end());
    blueMax = *std::max_element(blueHues.begin(), blueHues.end());
  }

  optical_sensor.set_led_pwm(0); //sets led brightness to 0%

  pros::delay(50); //delay calibration

  // Store or use blueMin and blueMax as needed
  printf("Blue Avg: %.2f\n", blueAvg);
  printf("Blue Min: %.2f\n", blueMin);
  printf("Blue Max: %.2f\n", blueMax);
}

// void newSAWP(){
//   chassis.drive_angle_set(90_deg); //start off facing 90 deg
//   chassis.pid_wait();
//   chassis.pid_drive_set(32.5_in,DRIVE_SPEED+5); //go towards goal 33 inches
//   chassis.pid_wait();
//   lilRaaahh.set(true); //deploy matchloader mechanism
//   chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader 
//   chassis.pid_wait();
//   middleIntakeMotor.move(127); //run first stage intake 
//   pros::delay(90); //delay to ensure mechanism deployed and intake running 120
//   chassis.pid_drive_set(10.5_in,BALLGOAL_SPEED+10+5); //go into matchloader 11.5 inches
//   chassis.pid_wait();
//   pros::delay(20); //amount of time delay to intake 3 balls
//   chassis.pid_drive_set(-31.5_in,DRIVE_SPEED); //go backwards into long goal by 34.5 inches 
//   pros::delay(1100); //short delay before running intake
//   intake.move(127); //run full intake to score 3 balls
//   lilRaaahh.set(false); //retract matchloader mechanism
//   pros::delay(500); //run intake for 600 ms to score balls

//   chassis.pid_drive_set(12_in,DRIVE_SPEED+5); //6
//   chassis.pid_wait();
//   chassis.pid_turn_set(303_deg,TURN_SPEED); //315
//   //chassis.pid_wait();
//   pros::delay(700);
//   middleIntakeMotor.move(127);
//   chassis.pid_drive_set(27_in, DRIVE_SPEED+5);
//   chassis.pid_wait_until(15_in);
//   chassis.pid_speed_max_set(BALLGOAL_SPEED-15);
//   chassis.pid_wait();

//   chassis.pid_turn_set(270_deg,TURN_SPEED); //turn towards next cluster of 3 balls
//   pros::delay(200);
//   chassis.pid_drive_set(50_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
//   chassis.pid_wait_until(27_in);//at 34 inches switch to ballgoalspeed -15

//   //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
//   chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
//   chassis.pid_wait();
//   chassis.pid_drive_set(-5_in, DRIVE_SPEED+5); //back away from cluster of balls 22 inches
//   pros::delay(300);
//   chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
//   chassis.pid_wait();
//   middleIntakeMotor.move(-30);
//   lilRaaahh.set(true);

//   chassis.pid_drive_set(-18_in,DRIVE_SPEED+5); //back into middle goal 
//   chassis.pid_wait();
//   middleGoalScore.set(true);
//   middleIntakeMotor.move(110); //run first stage intake at full
//   topIntakeMotor.move(-60); //run top stage intake forward at reduced speed
//   pros::delay(800); //200 ms delay for scoring 
//   middleGoalScore.set(false);
//   intake.move(0); //stop the intake

//   chassis.pid_turn_set(215_deg,TURN_SPEED); //face back of robot to middle goal
//   chassis.pid_wait();
  
//   chassis.pid_drive_set(50_in, DRIVE_SPEED+15); //drive toward field perimeter wall direction 53.5
//   chassis.pid_wait();
//   topIntakeMotor.move(0); //stop the top intake motor
//   chassis.pid_turn_set(180_deg, TURN_SPEED); //turn toward direction of the matchloader 
//   chassis.pid_wait();
//   middleIntakeMotor.move(127); //run first stage intake 
//   chassis.pid_drive_set(10_in,BALLGOAL_SPEED+10+5); //go into matchloader 11.5 inches
//   pros::delay(1100);

//   middleIntakeMotor.move(100); //run the bottom intake stage 
//   chassis.pid_drive_set(-30_in, DRIVE_SPEED+10+5); //go backwards into long goal 24
//   pros::delay(1100);
//   intake.move(127); //run the intake
//   pros::delay(1000); //score remaaining balls on the intake 
//   chassis.pid_drive_set(4_in, DRIVE_SPEED+20); //move forward so extra balls/robot not touching goal
//   chassis.pid_wait();


// }

void fourCamp(){
  chassis.drive_angle_set(-24.5_deg); //robot faces three cluster balls at 26 deg
  chassis.pid_wait();
  middleIntakeMotor.move(127); //bottom stage intake spins 
  chassis.pid_drive_set(31_in, DRIVE_SPEED+5); //drive 33 inches toward 3 cluster of balls at drive_speed-5
  chassis.pid_wait_until(15_in); //after 15 inches switch to ballgoalspeed-20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  chassis.pid_wait();
  pros::delay(100); //wait 300 ms to intake the balls
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait();
  middleIntakeMotor.move(-35); //middleInatkMotor and 20

  chassis.pid_drive_set(-19_in,DRIVE_SPEED); //drive backwards into goal by 19 inches
  chassis.pid_wait_until(2_in); //wait until 4 inches away from target to switch to ballgoal speed
  middleGoalScore.set(true); //deploy middle goal scoring piston
  chassis.pid_wait();

  //middleGoalScore.set(true); //deploy middle goal scoring piston 
  middleIntakeMotor.move(105);  //110
  topIntakeMotor.move(-60);
  pros::delay(1550); //time for middle goal scoring - 1560 for 3 balls and 1610 for 4 balls 
  chassis.pid_drive_set(2_in,DRIVE_SPEED); //drive forward 5 inches to unjam balls from goal
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in,DRIVE_SPEED); //drive back 5 inches into goal again
  chassis.pid_wait();
  middleGoalScore.set(false); //retract middle goal scoring piston 
  intake.move(0); //stop spinning the intake 

  chassis.pid_drive_set(14_in,DRIVE_SPEED); //12
  chassis.pid_wait();
  middleGoalDescore.set(true);
  chassis.pid_drive_set(-11_in,BALLGOAL_SPEED); //12
  chassis.pid_wait();
}

void leftSevenBallWing(){
  chassis.drive_angle_set(-26_deg); //was 26
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-25);
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5);
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_turn_set(228_deg, TURN_SPEED);   
  chassis.pid_wait();
  chassis.pid_drive_set(32.5_in,DRIVE_SPEED); //32.5 non issam  and ballgoal + 10
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  lilRaaahh.set(true);

  pros::delay(250);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-15);
  chassis.pid_drive_set(27_in, DRIVE_SPEED); //25 non issam
  chassis.pid_wait_until(5_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
  // chassis.pid_wait();
  pros::delay(960); //110
  intake.move(5); //-20

  chassis.pid_drive_set(-31_in,DRIVE_SPEED); //-33 regular not issaam
  chassis.pid_wait();
  intake.move(127);
  pros::delay(2000);
  intake.move(0);
  lilRaaahh.set(false);
  chassis.pid_drive_set(8_in,DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(115_deg,TURN_SPEED); //65
  chassis.pid_wait();
  chassis.pid_drive_set(12.5_in,DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED); //225
  chassis.pid_wait();
  chassis.pid_drive_set(-28_in,DRIVE_SPEED-5);  
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in,40);
  chassis.pid_wait();
}

void leftSevenBall(){
  chassis.drive_angle_set(-26_deg); //was 26
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5);
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(100);

  chassis.pid_turn_set(228_deg, TURN_SPEED);   
  chassis.pid_wait();
  chassis.pid_drive_set(34.5_in,BALLGOAL_SPEED+10); //32.5 non issam  and ballgoal + 10
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  lilRaaahh.set(true);

  pros::delay(250);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(0);
  chassis.pid_drive_set(26_in, DRIVE_SPEED); //25 non issam
  chassis.pid_wait_until(5_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
  chassis.pid_wait();
  pros::delay(85); //110
  intake.move(-20);

  chassis.pid_drive_set(-31_in,DRIVE_SPEED); //-33 regular not issaam
  chassis.pid_wait();
  intake.move(127);
  pros::delay(2000);
  intake.move(0);
  chassis.pid_drive_set(6_in,DRIVE_SPEED); //move forward 6 inches
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in,DRIVE_SPEED); //move back 8 inches to push into control zone 
  chassis.pid_wait();
  descore.set(true); //deploy descoring mechanism
  intake.move(127); //run intake to empty out remaining balls
}

void threePlusFour(){
  chassis.drive_angle_set(-24.5_deg); //robot faces three cluster balls at 26 deg
  chassis.pid_wait();
  middleIntakeMotor.move(127); //bottom stage intake spins 
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5); //drive 33 inches toward 3 cluster of balls at drive_speed-5
  chassis.pid_wait_until(15_in); //after 15 inches switch to ballgoalspeed-20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  chassis.pid_wait();
  pros::delay(100); //wait 300 ms to intake the balls
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait();
  intake.move(-20); //middleInatkMotor and 20

  chassis.pid_drive_set(-20_in,DRIVE_SPEED); //drive backwards into goal by 18 inches
  chassis.pid_wait_until(2_in); //wait until 4 inches away from target to switch to ballgoal speed
  middleGoalScore.set(true); //deploy middle goal scoring piston
  chassis.pid_wait();

  //middleGoalScore.set(true); //deploy middle goal scoring piston 
  middleIntakeMotor.move(90);  //110
  topIntakeMotor.move(-60);
  pros::delay(1600); //time for middle goal scoring - 1560 for 3 balls and 1610 for 4 balls 
  chassis.pid_drive_set(2_in,DRIVE_SPEED); //drive forward 5 inches to unjam balls from goal
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in,DRIVE_SPEED); //drive back 5 inches into goal again
  chassis.pid_wait();
  middleGoalScore.set(false); //retract middle goal scoring piston 
  intake.move(0); //stop spinning the intake 
  chassis.pid_drive_set(56.5_in, DRIVE_SPEED-5); //drive 55 inches towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader 
  chassis.pid_wait();
  // lilRaaahh.set(true); //deploy matchloader mechanism
  // pros::delay(250); //250 ms delay to wait until matchloader deploys
  middleIntakeMotor.move(100); //running the bottom intake stage to matchload
  topIntakeMotor.move(0);
  chassis.pid_drive_set(18_in, DRIVE_SPEED+10); //drive in by 16 incehs into matchloader 
  chassis.pid_wait_until(2_in); //wait for 4 inches before switching to ballgoal speed 
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5); 
  // chassis.pid_wait();
  pros::delay(940); //140 ms delay for matchloading 3 balls  15 ms 

  chassis.pid_drive_set(-32_in,DRIVE_SPEED-10); //drive back into long goal by 34 in
  //lilRaaahh.set(false); //retract matchloader mechanism
  middleIntakeMotor.move(30); //slightly run bottom stage back for jamming
  chassis.pid_wait();
  intake.move(127); //run full intake to score on the long goal
  pros::delay(1245); //1350 ms delay for long goal scoring //1350
  intake.move(0); //stop the intake from spinning 
  descore.set(false); //disable descore so it comes down 
  lilRaaahh.set(false); //disengage matchload mechanism
  chassis.pid_drive_set(8_in,DRIVE_SPEED); //drive forward 8 inches 
  chassis.pid_wait();
  chassis.pid_turn_set(115_deg,TURN_SPEED); //turn 115 deg to face other side perimeter 
  chassis.pid_wait();
  chassis.pid_drive_set(13_in,DRIVE_SPEED);  //drive diagonally 13 inches 
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED); //face parallel to the long goal
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in,BALLGOAL_SPEED+20); //move back into the long goal 2
  chassis.pid_wait();
  chassis.pid_drive_set(-40_in,40); //keep moving back passive to prevent other alliance balls
  chassis.pid_wait();
}

void rightSevenBallWing(){
  chassis.drive_angle_set(24.5_deg); //was 26
  chassis.pid_wait();
  topIntakeMotor.move(-10);
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5);
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(100); 

  chassis.pid_turn_set(138_deg, TURN_SPEED); //145 deg 
  chassis.pid_wait();
  chassis.pid_drive_set(37.5_in,DRIVE_SPEED); //drivespeed and 36.5
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  //lilRaaahh.set(true);

  pros::delay(250); //150
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(25_in, DRIVE_SPEED-5); //21 too short
  chassis.pid_wait_until(5_in); //7
  chassis.pid_speed_max_set(BALLGOAL_SPEED-32); //-10
  chassis.pid_wait();
  pros::delay(80); //85
  intake.move(-20);

  chassis.pid_drive_set(-30_in,DRIVE_SPEED); //-33 drive speed -20-10-10



  chassis.pid_wait();
  intake.move(127);
  pros::delay(2000);
  intake.move(0);
  descore.set(false); //deploy descoring mechanism
  intake.move(127); //run intake to empty out remaining balls
  // descore.set(false);
  lilRaaahh.set(false);
  chassis.pid_drive_set(8_in,DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(115_deg,TURN_SPEED); //65
  chassis.pid_wait();
  chassis.pid_drive_set(13.5_in,DRIVE_SPEED); 
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED); //225
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in,BALLGOAL_SPEED);  //slow: ballgoalspeed
  chassis.pid_wait();
  chassis.pid_drive_set(-40_in,40);
  chassis.pid_wait();
}

void rightSevenBall(){
  chassis.drive_angle_set(24.5_deg); //was 26
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5);
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(100); 

  chassis.pid_turn_set(138_deg, TURN_SPEED); //145 deg 
  chassis.pid_wait();
  chassis.pid_drive_set(37.5_in,DRIVE_SPEED); //drivespeed and 36.5
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  //lilRaaahh.set(true);

  pros::delay(250); //150
  middleIntakeMotor.move(127);
  topIntakeMotor.move(0);
  chassis.pid_drive_set(29_in, DRIVE_SPEED); //21 too short
  chassis.pid_wait_until(5_in); //7
  chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
  chassis.pid_wait();
  pros::delay(85); //150
  intake.move(-20);

  chassis.pid_drive_set(-30_in,DRIVE_SPEED); //-33 drive speed -20-10-10
  chassis.pid_wait();
  intake.move(127);
  pros::delay(2000);
  intake.move(0);

  chassis.pid_drive_set(6_in,DRIVE_SPEED); //move forward 6 inches
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in,DRIVE_SPEED); //move back 8 inches to push into control zone 
  chassis.pid_wait();
  descore.set(true); //deploy descoring mechanism
  intake.move(127); //run intake to empty out remaining balls
}

void leftFourBallWing(){
  // chassis.drive_angle_set(-26_deg); //start at negative 26 deg
  // chassis.pid_wait();
  // middleIntakeMotor.move(127);
  // chassis.pid_drive_set(32_in, DRIVE_SPEED+5); //for 15 inches go at drivespeed +5
  // chassis.pid_wait_until(15_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20); //after 15 inches go at ballgoalspeed -20
  // lilRaaahh.set(true); //deploy lil will

  // chassis.pid_wait();
  // pros::delay(60); //60 second delay for matchloading the three balls 

  // chassis.pid_turn_set(235_deg, TURN_SPEED); //turn towards the field perimeter wall
  // chassis.pid_wait();
  // chassis.pid_drive_set(32.5_in,DRIVE_SPEED+20); //32.5 inches towards the field perimeter wall 
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED); //face the matchloader 
  // chassis.pid_wait();

  // chassis.pid_drive_set(-15_in,DRIVE_SPEED+20); //drive back into the matchloader by 15 inches
  // chassis.pid_wait();
  // intake.move(127); //start intake
  // pros::delay(1300); //score the 4 balls 

  // intake.move(0); //stop intake
  // descore.set(false); //activate descore
  // lilRaaahh.set(false); //retract lil will
  // chassis.pid_drive_set(8_in,DRIVE_SPEED+20);  //drive 8 inches forward 
  // chassis.pid_wait();
  // chassis.pid_turn_set(115_deg,TURN_SPEED); //turn towrads field perimeter wall
  // chassis.pid_wait();
  // chassis.pid_drive_set(12.5_in,DRIVE_SPEED+20); //go forward 12.5 inches to perimeter wall
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg,TURN_SPEED); //align parallel to the long goal 
  // chassis.pid_wait();
  // chassis.pid_drive_set(-32_in,BALLGOAL_SPEED+30);  //slow: ballgoalspeed fast =30 drive back into long goal by 32 inches
  // chassis.pid_wait();
  // chassis.pid_drive_set(-40_in,40); //keep driving back 
  // chassis.pid_wait();

  chassis.drive_angle_set(-24.5_deg);  
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(30_in, DRIVE_SPEED); //32
  chassis.pid_wait_until(14_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(50);

  chassis.pid_turn_set(240_deg, TURN_SPEED);   
  chassis.pid_wait();
  chassis.pid_drive_set(30.5_in,DRIVE_SPEED-8-5); //32.5 non issam  and ballgoal + 10
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  lilRaaahh.set(true);
  middleIntakeMotor.move(30);
  chassis.pid_drive_set(-12_in,DRIVE_SPEED); //drive back into the matchloader by 15 inches
  pros::delay(500);
  intake.move(127); //start intake
  pros::delay(1350); //score the 4 balls 

  chassis.pid_drive_set(8_in,DRIVE_SPEED+20);  //drive 8 inches forward 
  pros::delay(700);
  chassis.pid_turn_set(115_deg,TURN_SPEED); //turn towrads field perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(13_in,DRIVE_SPEED+20); //go forward 12.5 inches to perimeter wall
  pros::delay(700);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //align parallel to the long goal 
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in,BALLGOAL_SPEED+30);  //slow: ballgoalspeed fast =30 drive back into long goal by 32 inches
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in,40); //keep driving back 
  chassis.pid_wait();
  lilRaaahh.set(false); //retract lil will

}

void rightFourBallWing(){
  chassis.drive_angle_set(24.5_deg); //was 26
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(33_in, DRIVE_SPEED+5); //32
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED);
  lilRaaahh.set(true);

  chassis.pid_wait();
  pros::delay(100); 
  //topIntakeMotor.move(-40);

  chassis.pid_turn_set(138_deg, TURN_SPEED); //145 deg 
  chassis.pid_wait();
  lilRaaahh.set(false);
  chassis.pid_drive_set(37.5_in,DRIVE_SPEED); //drivespeed and 36.5
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-21_in,DRIVE_SPEED); //drivespeed and 39.5
  chassis.pid_wait();
  intake.move(127);
  pros::delay(1100);

  intake.move(0);
  descore.set(false);
  lilRaaahh.set(false);
  chassis.pid_drive_set(8_in,DRIVE_SPEED); 
  pros::delay(700);
  chassis.pid_turn_set(115_deg,TURN_SPEED); //65
  chassis.pid_wait();
  chassis.pid_drive_set(12.5_in,DRIVE_SPEED); 
  pros::delay(700);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //225
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in,BALLGOAL_SPEED+30);  //slow: ballgoalspeed
  chassis.pid_wait();
  chassis.pid_drive_set(-40_in,40);
  chassis.pid_wait();
}

void sawp(){
  chassis.drive_angle_set(90_deg); //start off facing 90 deg
  chassis.pid_wait();
  chassis.pid_drive_set(31.5_in,DRIVE_SPEED+5); //go towards goal 33 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader 
  chassis.pid_wait();
  middleIntakeMotor.move(127); //run first stage intake 
  pros::delay(90); //delay to ensure mechanism deployed and intake running 120
  chassis.pid_drive_set(10.5_in,BALLGOAL_SPEED+10+5); //go into matchloader 11.5 inches
  chassis.pid_wait();
  pros::delay(30); //amount of time delay to intake 3 balls
  chassis.pid_drive_set(-32.5_in,DRIVE_SPEED+5); //go backwards into long goal by 34.5 inches 
  pros::delay(1100); //short delay before running intake
  intake.move(127); //run full intake to score 3 balls
  lilRaaahh.set(false); //retract matchloader mechanism
  pros::delay(600); //run intake for 600 ms to score balls
  chassis.drive_angle_set(180_deg);
  chassis.pid_turn_set(280_deg,TURN_SPEED-30); //turn towards middle goal 
  chassis.pid_wait();
  topIntakeMotor.move(0); //turn of top intake motor
  middleIntakeMotor.move(127); //keep running first stage intake
  chassis.pid_drive_set(18_in, DRIVE_SPEED+5); //drive towards middle goal 16 inches
  chassis.pid_wait_until(4_in); //wait until 4 inches away from goal and then switch o ballgoalspeed -15
  //lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg,TURN_SPEED); //turn towards next cluster of 3 balls
  pros::delay(200);
  lilRaaahh.set(false); //deplpy matchloader mechanism to pick up balls 
  chassis.pid_drive_set(50_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(27_in);//at 34 inches switch to ballgoalspeed -15
  //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  chassis.pid_speed_max_set(BALLGOAL_SPEED-25);
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED+5); //back away from cluster of balls 22 inches
  pros::delay(200);
  chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in,DRIVE_SPEED+5); //back into middle goal 
  chassis.pid_wait();
  middleIntakeMotor.move(127); //run first stage intake at full
  topIntakeMotor.move(60); //run top stage intake forward at reduced speed
  pros::delay(200); //200 ms delay for scoring 
  intake.move(0); //stop the intake
  lilRaaahh.set(false); //set the matchloader mechanism to retracted
  chassis.pid_drive_set(51.5_in, DRIVE_SPEED+15); //drive toward field perimeter wall direction 53.5
  chassis.pid_wait();
  topIntakeMotor.move(0); //stop the top intake motor
  chassis.pid_turn_set(180_deg, TURN_SPEED); //turn toward direction of the matchloader 
  chassis.pid_wait();
  middleIntakeMotor.move(100); //run the bottom intake stage 
  chassis.pid_drive_set(-25_in, DRIVE_SPEED+10); //go backwards into long goal 24
  chassis.pid_wait();
  intake.move(127); //run the intake
  pros::delay(1000); //score remaaining balls on the intake 
  chassis.pid_drive_set(4_in, DRIVE_SPEED+20); //move forward so extra balls/robot not touching goal
  chassis.pid_wait();
}

void newSAWP(){
  chassis.drive_angle_set(90_deg); //start off facing 90 deg
  chassis.pid_wait();
  chassis.pid_drive_set(32.5_in,DRIVE_SPEED+5); //go towards goal 33 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader 
  chassis.pid_wait();
  pros::delay(50);
  middleIntakeMotor.move(127); //run first stage intake 
  topIntakeMotor.move(-20); //15
  pros::delay(85); //delay to ensure mechanism deployed and intake running 90
  chassis.pid_drive_set(11.5_in,BALLGOAL_SPEED+10); //go into matchloader 10.5 inches
  //chassis.pid_wait();
  pros::delay(1100); //amount of time delay to intake 3 balls 4 975
  middleIntakeMotor.move(100); //run the bottom intake stage 
  topIntakeMotor.move(-5);
  
  chassis.pid_drive_set(-32.5_in,DRIVE_SPEED); //go backwards into long goal by 34.5 inches 
  pros::delay(1080); //short delay before running intake //1090
  intake.move(127); //run full intake to score 3 balls
  lilRaaahh.set(false); //retract matchloader mechanism
  pros::delay(670); //run intake for 650 ms to score balls
  chassis.drive_angle_set(180_deg);
  // chassis.drive_angle_set(180_deg); //start off facing 90 deg
  // chassis.pid_wait();
  chassis.pid_turn_set(280_deg,TURN_SPEED); //turn towards middle goal 
  chassis.pid_wait();
  topIntakeMotor.move(0); //turn of top intake motor
  middleIntakeMotor.move(127); //keep running first stage intake
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(18_in, DRIVE_SPEED+5); //drive towards middle goal 16 inches
  chassis.pid_wait_until(4_in); //wait until 4 inches away from goal and then switch o ballgoalspeed -15
  //lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
  chassis.pid_wait();
  chassis.pid_turn_set(270_deg,TURN_SPEED); //turn towards next cluster of 3 balls
  pros::delay(200);
  chassis.pid_drive_set(50_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(27_in);//at 34 inches switch to ballgoalspeed -15

  //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  chassis.pid_speed_max_set(BALLGOAL_SPEED-10);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED+5); //back away from cluster of balls -5 inches
  pros::delay(300);
  chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
  chassis.pid_wait();
  topIntakeMotor.move(-20); //0
  middleIntakeMotor.move(0); //-30
  lilRaaahh.set(true);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED+5); //back into middle goal 21
  // chassis.pid_wait();
  chassis.pid_wait_until(-10); //-6
  middleGoalScore.set(true);
  middleIntakeMotor.move(110); //run first stage intake at full
  topIntakeMotor.move(-60); //run top stage intake forward at reduced speed initally out 
  pros::delay(805); //200 ms delay for scoring  800
  middleGoalScore.set(false);
  // intake.move(0); //stop the intake
  middleIntakeMotor.move(0);
  topIntakeMotor.move(-10);
  //lilRaaahh.set(false); //set the matchloader mechanism to retracted
  chassis.pid_drive_set(53.5_in, DRIVE_SPEED+10); //drive toward field perimeter wall direction 54.5
  chassis.pid_wait();
  topIntakeMotor.move(0); //stop the top intake motor
  chassis.pid_turn_set(180_deg, TURN_SPEED); //turn toward direction of the matchloader 
  chassis.pid_wait();
  middleIntakeMotor.move(127); //run first stage intake 
  pros::delay(50);
  chassis.pid_drive_set(13_in,BALLGOAL_SPEED+10+5); //go into matchloader 13.5 inches
  pros::delay(1000); //1100

  middleIntakeMotor.move(100); //run the bottom intake stage 
  chassis.pid_drive_set(-31_in, DRIVE_SPEED+10+5); //go backwards into long goal 30
  pros::delay(990); //1100
  intake.move(127); //run the intake
  pros::delay(1200); //score remaaining balls on the intake 
  chassis.pid_drive_set(4_in, DRIVE_SPEED+20); //move forward so extra balls/robot not touching goal
  chassis.pid_wait();


}

void antiAWP(){
  // chassis.drive_angle_set(-24.5_deg); //robot faces three cluster balls at 24.5 deg
  // chassis.pid_wait();
  // middleIntakeMotor.move(127); //bottom stage intake spins 
  // chassis.pid_drive_set(31_in, DRIVE_SPEED+5); //drive 31 inches toward 3 cluster of balls at drive_speed+5
  // chassis.pid_wait_until(15_in); //after 15 inches switch to ballgoalspeed-20
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  // chassis.pid_wait();
  // pros::delay(100); //wait 100 ms to intake the balls
  // chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  // chassis.pid_wait();
  // middleIntakeMotor.move(-30); //middleIntakeMotor moves backward 
  // middleGoalScore.set(true); //deploy middle goal 
  // chassis.pid_drive_set(-20_in,DRIVE_SPEED+5); //drive backwards into goal by 20 inches
  // chassis.pid_wait();
  // middleIntakeMotor.move(90);  //controlled scoring 
  // topIntakeMotor.move(-60);
  // pros::delay(1560); //time for middle goal scoring - 1560 for 3 balls and 1610 for 4 balls 
  // middleGoalScore.set(false); //retract middle goal scoring piston 
  // intake.move(0); //stop spinning the intake 
  // chassis.pid_drive_set(56_in, DRIVE_SPEED+15); //drive 56 inches towards field perimeter wall
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader 180 deg
  // chassis.pid_wait();
  // middleIntakeMotor.move(100); //running the bottom intake stage to matchload 100 
  // topIntakeMotor.move(0); //stop top stage 
  // chassis.pid_drive_set(17_in, DRIVE_SPEED+10); //drive in by 17 incehs into matchloader 
  // chassis.pid_wait_until(2_in); //wait for 2 inches before switching to ballgoal speed 
  // chassis.pid_speed_max_set(BALLGOAL_SPEED); 
  // chassis.pid_wait();
  // pros::delay(90); //90 ms delay for matchloading 3 balls
  // chassis.pid_drive_set(-35_in,DRIVE_SPEED); //drive back into long goal by 34 in
  // middleIntakeMotor.move(-10); //slightly run bottom stage back for jamming
  // chassis.pid_wait();
  // intake.move(127); //run full intake to score on the long goal
  // pros::delay(1400); //1400 ms delay for long goal scoring
  // intake.move(0); //stop the intake from spinning 
  // descore.set(false); //disable descore so it comes down 
  // lilRaaahh.set(false); //disengage matchload mechanism
  // chassis.pid_drive_set(20_in, DRIVE_SPEED+15); //drive 20 inches towards field perimeter wall
  // chassis.pid_wait();
  // chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  // chassis.pid_wait();
  // middleGoalDescore.set(true); //deploy middle goal scoring piston
  // chassis.pid_drive_set(-55_in, DRIVE_SPEED-25); //drive in by -45 incehs into matchloader 
  // chassis.pid_wait_until(-45_in); //wait for 45 inches before switching to drivespeed - 15
  // chassis.pid_speed_max_set(DRIVE_SPEED-15); 
  // chassis.pid_wait();
  // chassis.pid_drive_set(37_in, DRIVE_SPEED); //drive 37 inches towards field perimeter wall 
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards long goal
  // chassis.pid_wait();
  // descore.set(true); //engage descore for long goal 

  chassis.drive_angle_set(-24.5_deg); //robot faces three cluster balls at 26 deg
  chassis.pid_wait();
  middleIntakeMotor.move(127); //bottom stage intake spins 
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5); //drive 33 inches toward 3 cluster of balls at drive_speed-5
  chassis.pid_wait_until(15_in); //after 15 inches switch to ballgoalspeed-20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  chassis.pid_wait();
  pros::delay(100); //wait 300 ms to intake the balls
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait();
  middleIntakeMotor.move(-35); //middleInatkMotor and 20

  chassis.pid_drive_set(-19_in,DRIVE_SPEED); //drive backwards into goal by 18 inches
  chassis.pid_wait_until(2_in); //wait until 4 inches away from target to switch to ballgoal speed
  middleGoalScore.set(true); //deploy middle goal scoring piston
  chassis.pid_wait();

  //middleGoalScore.set(true); //deploy middle goal scoring piston 
  middleIntakeMotor.move(105);  //110
  topIntakeMotor.move(-60);
  pros::delay(1515); //time for middle goal scoring - 1560 for 3 balls and 1610 for 4 balls 
  chassis.pid_drive_set(2_in,DRIVE_SPEED); //drive forward 5 inches to unjam balls from goal
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in,DRIVE_SPEED); //drive back 5 inches into goal again
  chassis.pid_wait();
  middleGoalScore.set(false); //retract middle goal scoring piston 
  intake.move(0); //stop spinning the intake 
  chassis.pid_drive_set(56_in, DRIVE_SPEED-5); //drive 55 inches towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader 
  chassis.pid_wait();
  // lilRaaahh.set(true); //deploy matchloader mechanism
  // pros::delay(250); //250 ms delay to wait until matchloader deploys
  middleIntakeMotor.move(100); //running the bottom intake stage to matchload
  topIntakeMotor.move(0);
  chassis.pid_drive_set(19_in, DRIVE_SPEED+10); //drive in by 16 incehs into matchloader 18
  chassis.pid_wait_until(2_in); //wait for 4 inches before switching to ballgoal speed 
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5); 
  chassis.pid_wait();
  pros::delay(10); //140 ms delay for matchloading 3 balls  15 ms 

  chassis.pid_drive_set(-32_in,DRIVE_SPEED-10); //drive back into long goal by 34 in
  //lilRaaahh.set(false); //retract matchloader mechanism
  middleIntakeMotor.move(30); //slightly run bottom stage back for jamming
  chassis.pid_wait();
  intake.move(127); //run full intake to score on the long goal
  pros::delay(900); //1350 ms delay for long goal scoring
  intake.move(0); //stop the intake from spinning 
  descore.set(false); //disable descore so it comes down 
  lilRaaahh.set(false); //disengage matchload mechanism

  chassis.pid_drive_set(20_in, DRIVE_SPEED+15); //drive 20 inches towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait();
  middleGoalDescore.set(true); 
  chassis.pid_drive_set(-45_in,DRIVE_SPEED-25); //drive back 5 inches into goal again
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in,DRIVE_SPEED-20); //drive back 5 inches into goal again -20 weak and 15 strong
  chassis.pid_wait();
  chassis.pid_drive_set(37_in, DRIVE_SPEED); //drive 37 inches towards field perimeter wall 
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards long goal
  chassis.pid_wait();
  descore.set(true); //engage descore for long goal 
}
void park7() {
  // // chassis.drive_angle_set(0_deg); //face forward
  // // chassis.pid_wait();
  // // middleIntakeMotor.move(127);
  // // chassis.pid_drive_set(60_in, 55); //drive forward 60 inches into the parking zone
  // // chassis.pid_wait();
  // topIntakeMotor.move(-20);
  // chassis.pid_drive_set(21_in,DRIVE_SPEED); //7
  // chassis.pid_wait();
  // chassis.pid_turn_set(63_deg, TURN_SPEED); //40
  // chassis.pid_wait();
  // chassis.pid_drive_set(30_in,DRIVE_SPEED); //33 +10 and 42
  // chassis.pid_wait();
  // middleIntakeMotor.move(127);
  // chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  // //chassis.pid_turn_set(89_deg, TURN_SPEED); //turn towards field perimeter wall
  // chassis.pid_wait();
  // chassis.pid_drive_set(50_in, 69); //drive forward 60 inches into the parking zone
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED); //220
  // chassis.pid_wait();
  // chassis.pid_drive_set(-6_in, 40); //drive back 20 inches to be fully in the parking zone
  // chassis.pid_wait();
  // topIntakeMotor.move(-20);
  // // chassis.pid_turn_set(225_deg, TURN_SPEED); //220
  // // chassis.pid_wait();
  // chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  // chassis.pid_wait();
  // //middleIntakeMotor.move(-10);
  // topIntakeMotor.move(-60);
  // chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  // chassis.pid_wait();
  // lilRaaahh.set(true);
  // middleIntakeMotor.move(-45);
  // topIntakeMotor.move(-70);
  // chassis.pid_drive_set(-18_in, DRIVE_SPEED); //drive back 23
  // chassis.pid_wait_until(-15_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // middleGoalScore.set(true);
  // chassis.pid_wait();
  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(-120); //-60 top 
  // pros::delay(4000); //3500
  // chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // middleIntakeMotor.move(80); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(1000); //3500

  //new
  //park and mid 7
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(21_in,DRIVE_SPEED); //7
  chassis.pid_wait();
  chassis.pid_turn_set(63_deg, TURN_SPEED); //40
  chassis.pid_wait();
  chassis.pid_drive_set(30_in,DRIVE_SPEED); //33 +10 and 42
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  chassis.pid_wait();
  chassis.pid_drive_set(50_in, 69); //drive forward 60 inches into the parking zone
  chassis.pid_wait();

  chassis.pid_drive_set(11_in, 69); //drive to collect balls fallen balls 
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, 69); //drive back to be in line with middle goal
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, 40); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  chassis.pid_wait();
  topIntakeMotor.move(-60);
  chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  chassis.pid_wait();
  lilRaaahh.set(true);
  // middleIntakeMotor.move(-45);
  // topIntakeMotor.move(-70);
  intake.move(-40);
  chassis.pid_drive_set(-18_in, DRIVE_SPEED); //drive back 23
  chassis.pid_wait_until(-9_in);
  middleGoalScore.set(true);
  chassis.pid_wait_until(-15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait();
  // Start a slow passive drive backwards into the goal while scoring to help align
  middleIntakeMotor.move(110); //90 bottom 
  topIntakeMotor.move(-60); //-60 top 
  chassis.pid_drive_set(-8_in, 40); // increase distance and speed so robot actually moves while scoring
  pros::delay(4000); //3500
  chassis.pid_drive_set(2_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  chassis.pid_wait();
  middleIntakeMotor.move(80); //90 bottom 
  topIntakeMotor.move(0); //-60 top 
  pros::delay(1000); //3500
  intake.move(127);
}

void straightSAWP(){
  // chassis.drive_angle_set(180_deg); //start off facing 90 deg
  // chassis.pid_wait();
  // chassis.pid_turn_set(280_deg,TURN_SPEED); //turn towards middle goal 
  // chassis.pid_wait();
  // topIntakeMotor.move(0); //turn of top intake motor
  // middleIntakeMotor.move(127); //keep running first stage intake
  // topIntakeMotor.move(-20);
  // chassis.pid_drive_set(18_in, DRIVE_SPEED); //drive back 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // chassis.pid_turn_set(270_deg,TURN_SPEED); //turn towards next cluster of 3 balls
  // pros::delay(200);
  // chassis.pid_drive_set(50_in, DRIVE_SPEED); //drive towards cluster of balls 48 inches
  // chassis.pid_wait();
  // chassis.pid_drive_set(-4_in, DRIVE_SPEED+5); //back away from cluster of balls -5 inches
  // pros::delay(300);
  // chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
  // chassis.pid_wait();
  // topIntakeMotor.move(-20); //0
  // middleIntakeMotor.move(0); //-30
  // lilRaaahh.set(true);

  // chassis.pid_drive_set(-22_in,DRIVE_SPEED+5); //back into middle goal 21
  // // chassis.pid_wait();
  // chassis.pid_wait_until(-10); //-6
  // middleGoalScore.set(true);
  // middleIntakeMotor.move(110); //run first stage intake at full
  // topIntakeMotor.move(-60); //run top stage intake forward at reduced speed initally out 
  // pros::delay(805); //200 ms delay for scoring  800
  // middleGoalScore.set(false);
  // // intake.move(0); //stop the intake
  // middleIntakeMotor.move(0);
  // topIntakeMotor.move(-10);

  chassis.drive_angle_set(270_deg); //start off facing 90 deg
  chassis.pid_wait();
  middleIntakeMotor.move(127); //keep running first stage intake
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(68_in, DRIVE_SPEED); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(55_in);//at 34 inches switch to ballgoalspeed -15
  lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  chassis.pid_wait();
  lilRaaahh.set(true);

  chassis.pid_drive_set(-4_in, DRIVE_SPEED+5); //back away from cluster of balls -5 inches
  pros::delay(300);

  chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
  chassis.pid_wait();
  topIntakeMotor.move(-20); //0
  middleIntakeMotor.move(0); //-30
  lilRaaahh.set(true);
  
}

void fourSAWP(){
  chassis.drive_angle_set(90_deg); //start off facing 90 deg
  chassis.pid_wait();
  chassis.pid_drive_set(32.5_in,DRIVE_SPEED+5); //go towards goal 33 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader 
  chassis.pid_wait();
  pros::delay(50);
  middleIntakeMotor.move(127); //run first stage intake 
  topIntakeMotor.move(-20); //15
  pros::delay(85); //delay to ensure mechanism deployed and intake running 90
  chassis.pid_drive_set(11.5_in,BALLGOAL_SPEED+10); //go into matchloader 10.5 inches
  //chassis.pid_wait();
  pros::delay(950); //amount of time delay to intake 3 balls 4 975
  middleIntakeMotor.move(100); //run the bottom intake stage 
  topIntakeMotor.move(-5);
  
  chassis.pid_drive_set(-32.5_in,DRIVE_SPEED); //go backwards into long goal by 34.5 inches 
  pros::delay(950); //short delay before running intake //1090
  intake.move(127); //run full intake to score 3 balls
}

void optimizedSAWP(){
  chassis.drive_angle_set(90_deg); //start off facing 90 deg
  chassis.pid_wait();
  chassis.pid_drive_set(34.5_in,DRIVE_SPEED+5); //go towards goal 33 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader 
  chassis.pid_wait();
  pros::delay(50);
  middleIntakeMotor.move(127); //run first stage intake 
  topIntakeMotor.move(-20); //15
  pros::delay(85); //delay to ensure mechanism deployed and intake running 90
  chassis.pid_drive_set(11.5_in,BALLGOAL_SPEED+6); //go into matchloader 10.5 inches
  //chassis.pid_wait();
  pros::delay(950); //amount of time delay to intake 3 balls 4 975
  middleIntakeMotor.move(100); //run the bottom intake stage 
  topIntakeMotor.move(-5);
  
  chassis.pid_drive_set(-32.5_in,DRIVE_SPEED); //go backwards into long goal by 34.5 inches 
  pros::delay(950); //short delay before running intake //1090
  intake.move(127); //run full intake to score 3 balls
  lilRaaahh.set(false); //retract matchloader mechanism
  pros::delay(1050); //run intake for 650 ms to score balls
  chassis.drive_angle_set(180_deg);
  // chassis.drive_angle_set(180_deg); //start off facing 90 deg
  // chassis.pid_wait();
  chassis.pid_turn_set(290_deg,TURN_SPEED); //turn towards middle goal 280
  chassis.pid_wait();
  topIntakeMotor.move(0); //turn of top intake motor
  middleIntakeMotor.move(127); //keep running first stage intake
  topIntakeMotor.move(-20);

  chassis.pid_drive_set(5_in,DRIVE_SPEED); //go into matchloader 10.5 inches
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,TURN_SPEED); //start off facing 90 deg
  chassis.pid_wait();
  chassis.pid_drive_set(57_in, DRIVE_SPEED); //drive towards cluster of balls 58 inches
  chassis.pid_wait_until(44_in);//at 34 inches switch to ballgoalspeed -15
  lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  chassis.pid_wait();

  //chassis.pid_drive_set(-2_in, DRIVE_SPEED+5); //back away from cluster of balls -5 inches
  //chassis.pid_wait();
  
  chassis.pid_turn_set(225_deg,TURN_SPEED); //face back of robot to middle goal
  chassis.pid_wait();
  //topIntakeMotor.move(-90); //0
  //middleIntakeMotor.move(-45); //-30
  //topIntakeMotor.move(-90); //0
  //middleIntakeMotor.move(-30); //-30
  lilRaaahh.set(true);
  
  topIntakeMotor.move(-120); //0
  middleIntakeMotor.move(-50); //-30
  lilRaaahh.set(true);

  chassis.pid_drive_set(-20_in,DRIVE_SPEED); //back into middle goal 21
  // chassis.pid_wait();
  chassis.pid_wait_until(-11); //-6
  //chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  middleGoalScore.set(true);
  middleIntakeMotor.move(110); //run first stage intake at full 110
  topIntakeMotor.move(-60); //run top stage intake forward at reduced speed initally out
  chassis.pid_wait();

  pros::delay(800); //200 ms delay for scoring  800
  middleGoalScore.set(false);
  // intake.move(0); //stop the intake
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  //lilRaaahh.set(false); //set the matchloader mechanism to retracted
  chassis.pid_drive_set(53.5_in, DRIVE_SPEED); //drive toward field perimeter wall direction 54.5
  chassis.pid_wait();
  topIntakeMotor.move(0); //stop the top intake motor
  chassis.pid_turn_set(180_deg, TURN_SPEED); //turn toward direction of the matchloader 
  chassis.pid_wait();
  middleIntakeMotor.move(127); //run first stage intake 
  pros::delay(50);
  chassis.pid_drive_set(14_in,BALLGOAL_SPEED+4); //go into matchloader 13.5 inches
  pros::delay(980); //1100

  middleIntakeMotor.move(100); //run the bottom intake stage 
  chassis.pid_drive_set(-31_in, DRIVE_SPEED+10+5); //go backwards into long goal 30
  pros::delay(990); //1100
  intake.move(127); //run the intake
  pros::delay(1300); //score remaaining balls on the intake 
  chassis.pid_drive_set(4_in, DRIVE_SPEED+20); //move forward so extra balls/robot not touching goal
  chassis.pid_wait();

}

void mid7(){
  // chassis.drive_angle_set(-24.5_deg);  
  // chassis.pid_wait();
  // middleIntakeMotor.move(127);
  // topIntakeMotor.move(-20);
  // chassis.pid_drive_set(30_in, DRIVE_SPEED); //32
  // chassis.pid_wait_until(15_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // lilRaaahh.set(true);

  // chassis.pid_wait();
  // pros::delay(50);

  // chassis.pid_turn_set(223_deg, TURN_SPEED);   
  // chassis.pid_wait();
  // chassis.pid_drive_set(38_in,DRIVE_SPEED); //32.5 non issam  and ballgoal + 10
  // chassis.pid_wait();
  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
  // middleIntakeMotor.move(127);
  // topIntakeMotor.move(0);

  // chassis.pid_drive_set(16,DRIVE_SPEED-10); //33
  

  //  //forward 37 towards matchloader 
  // pros::delay(950);

  // chassis.pid_drive_set(-11,DRIVE_SPEED); //9.5
  // chassis.pid_wait();
  // //lilRaaahh.set(false);
  // chassis.pid_turn_set(226_deg, TURN_SPEED);
  // chassis.pid_wait();

  // // chassis.pid_drive_set(-54_in, DRIVE_SPEED-5);
  // middleIntakeMotor.move(0);
  // topIntakeMotor.move(-127);
  // // topIntakeMotor.move(-20);
  // chassis.pid_drive_set(-55_in, DRIVE_SPEED); //55
  // chassis.pid_wait_until(-40_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED);
  //  //drive 55 inches towards field perimeter wall
  // chassis.pid_wait_until(-25_in); //-30
  // middleGoalScore.set(true); 
  // chassis.pid_wait();

  // middleIntakeMotor.move(110);  //110
  // topIntakeMotor.move(-120);
  // pros::delay(2500);


  middleIntakeMotor.move(127);
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(21_in,DRIVE_SPEED); //7
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED); //40
  chassis.pid_wait();
  chassis.pid_drive_set(31_in,DRIVE_SPEED); //33 +10 and 42
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 75); //drive forward 60 inches into the parking zone
  //chassis.pid_wait_until(15_in); //wait until 10 inches away from target to switch to ballgoal speed
  //chassis.pid_speed_max_set(60);
  chassis.pid_wait();

  chassis.pid_drive_set(-14, DRIVE_SPEED-40); //drive to collect balls fallen balls 
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, DRIVE_SPEED); //drive back to be in line with middle goal
  chassis.pid_wait();
  //chassis.pid_drive_set(-20_in, 69); //drive back to be in line with middle goal
  //chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, 40); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  chassis.pid_wait();
  middleIntakeMotor.move(-20);
  topIntakeMotor.move(-127);
  chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  chassis.pid_wait();
  lilRaaahh.set(true);
  //intake.move(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED); //drive back 23
  chassis.pid_wait_until(-15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  middleGoalScore.set(true);
  chassis.pid_wait();
  antiJam = new pros::Task(doIntakeUnstuck);
  middleIntakeMotor.move(85); //90 bottom 
  topIntakeMotor.move(-70); //-60 top 
  chassis.pid_drive_set(-20_in,50); //towards goal 30 inches
  chassis.pid_wait();
  pros::delay(3000); //3500
  middleIntakeMotor.move(100); //90 bottom 
  topIntakeMotor.move(0); //-60 top 
  pros::delay(1000); //3500
  //chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  //chassis.pid_wait();
  //chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  //chassis.pid_wait();

  //7 middle from before version
  // pros::delay(4000); //3500
  // chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // middleIntakeMotor.move(80); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(1000); //3500
  //middleIntakeMotor.move(-50); //run the intake to score any remaining balls in the goal
  //pros::delay(1000); //3500
  //middleIntakeMotor.move(127); //stop the intake from spinning
  pros::delay(1000);
  // chassis.pid_turn_set(30_deg, TURN_SPEED); //turn towards field perimeter wall
  // pros::delay(400);
  // chassis.pid_turn_set(60_deg, TURN_SPEED); //turn towards matchloader
  // pros::delay(400);
  // chassis.pid_turn_set(45_deg, TURN_SPEED); //turn towards field perimeter wall
  // chassis.pid_wait();
  //chassis.pid_drive_set(2_in, DRIVE_SPEED); //drive forward 20 inches to be fully in the parking zone
  //chassis.pid_wait();
  //middleGoalScore.set(false); //retract middle goal scoring piston
  chassis.pid_drive_set(3_in, DRIVE_SPEED); //drive forward 50 inches into the parking zone
  pros::delay(300);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED); //drive back 50 inches into the parking zone
  chassis.pid_wait();

  pros::delay(1500);

  
  antiJam->remove();
  delete antiJam;
  antiJam = nullptr;

  chassis.pid_drive_set(12_in, DRIVE_SPEED); //drive forward 23 inches to be fully in the parking zone
  intake.move(127); //20c
  
  lilRaaahh.set(false);
  
}

void stateSkills(){
  chassis.drive_angle_set(0_deg); //face forward
  chassis.pid_wait();
  descore.set(true); //make sure descore is retracted at beginning of match
  middleIntakeMotor.move(127); //bottom stage intake spins 
  topIntakeMotor.move(-100); //top stage intake spins at reduced speed
  chassis.pid_turn_set(330_deg,TURN_SPEED); //turn towards matchloader -24.5
  chassis.pid_wait();
  chassis.pid_drive_set(32_in, DRIVE_SPEED+5); //drive 33 inches toward 3 cluster of balls at drive_speed-5
  chassis.pid_wait_until(14_in); //after 15 inches switch to ballgoalspeed-20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-35);
  chassis.pid_wait_until(16_in); //after 20 inches switch to ballgoalspeed-20
  //lilRaaahh.set(true); //deploy matchloader mechanism to pick up balls
  //lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED+5); //back away from cluster of balls -5 inches
  chassis.pid_wait();
  pros::delay(30); //wait 100 ms to intake the balls
  chassis.pid_turn_set(226_deg,TURN_SPEED); //turn 226 deg with back of robot facing middle goal
  chassis.pid_wait();
  middleIntakeMotor.move(-35); //middleInatkMotor and 20

  chassis.pid_drive_set(-19_in,DRIVE_SPEED); //drive backwards into goal by 18 inches
  chassis.pid_wait_until(2_in); //wait until 4 inches away from target to switch to ballgoal speed
  middleGoalScore.set(true); //deploy middle goal scoring piston
  chassis.pid_wait();

  middleIntakeMotor.move(100);  //110
  topIntakeMotor.move(-60);
  pros::delay(1600); //time for middle goal scoring - 1515 for 3 balls and 1610 for 4 balls 

  middleGoalScore.set(false); //retract middle goal scoring piston 
  intake.move(127); //stop spinning the intake 
  chassis.pid_drive_set(54_in, DRIVE_SPEED-5-5); //drive 55 inches towards field perimeter wall
  chassis.pid_wait();
  lilRaaahh.set(true); //deploy matchloader mechanism
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader 
  chassis.pid_wait();


  middleIntakeMotor.move(127); //running the bottom intake stage to matchload
  topIntakeMotor.move(-15);
  chassis.pid_drive_set(19_in, DRIVE_SPEED+10); //drive in by 16 incehs into matchloader 
  chassis.pid_wait_until(2_in); //wait for 4 inches before switching to ballgoal speed 
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5); 
  // chassis.pid_wait();
  pros::delay(800);

  //wiggle bot side to side in matchloader
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  // for (int i = 0; i < 3; i++) {
  //   chassis.pid_drive_set(2_in, BALLGOAL_SPEED-30); //back 1 in from matchload
  //   pros::delay(150);
  //   chassis.pid_turn_set(177.5_deg, TURN_SPEED);
  //   pros::delay(100);
  //   chassis.pid_turn_set(183.5_deg, TURN_SPEED);
  //   pros::delay(100);
  // }
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(600); //time for matchloading 700 with drive in out 
  chassis.pid_drive_set(-17_in, DRIVE_SPEED+10); //backward to goal 17 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract

  // //turning to other side/going to other side
  chassis.pid_turn_set(300_deg,TURN_SPEED+10); //300 deg to perimeter wall
  //chassis.pid_wait();
  pros::delay(800); //1000
  intake.move(0); //intake stop spinning
  chassis.pid_drive_set(15_in, DRIVE_SPEED+10); //16 in towards perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(64_in,DRIVE_SPEED-10-5); //go towards other side 65 in
  chassis.pid_wait();

  // //next quadrant
  chassis.pid_turn_set(53_deg,TURN_SPEED+10); //turn 50 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(16_in,DRIVE_SPEED+10); //17 inches diagonal movement to long goal
  pros::delay(650);
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards matchload with bot facing forwards 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-14_in,DRIVE_SPEED+10); //backward to long goal by 15
  pros::delay(500);
  intake.move(127); //intaking at goals
  pros::delay(1950); //delay for scoring at long goal 
  intake.move(0);
  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-15);
  pros::delay(100);

  chassis.pid_drive_set(45_in,DRIVE_SPEED); //forward 43
  chassis.pid_wait_until(15_in); //at 12 inches switch to ballgoalspeed+10
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5);
  pros::delay(1600);

  //back and forth in matchloader
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();

  // for (int i = 0; i < 3; i++) {
  //   chassis.pid_drive_set(2_in, BALLGOAL_SPEED-30); //back 1 in from matchload
  //   pros::delay(150);
  //   chassis.pid_turn_set(-3.5_deg, TURN_SPEED);
  //   pros::delay(100);
  //   chassis.pid_turn_set(3.5_deg, TURN_SPEED);
  //   pros::delay(100);
  // }
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(600); //time for matchloading 700 with drive in out
  intake.move(-5); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-33_in,DRIVE_SPEED+10); //backwards 33 
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(100);
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //300 deg to perimeter wall
  pros::delay(2400); //delay for scoring at long goal
  //chassis.pid_drive_set(4_in,127);//back forth goals for control 4 out 
  //chassis.pid_wait();
  //chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  //chassis.pid_wait();
  //middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(-20);
  lilRaaahh.set(false); //retract matchloader mech

  //park and mid 7 old

  // chassis.pid_drive_set(21_in,DRIVE_SPEED); //7
  // chassis.pid_wait();
  // chassis.pid_turn_set(63_deg, TURN_SPEED); //40
  // chassis.pid_wait();
  // chassis.pid_drive_set(30_in,DRIVE_SPEED); //33 +10 and 42
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  // chassis.pid_wait();
  // chassis.pid_drive_set(56_in, 75); //drive forward 60 inches into the parking zone
  // //chassis.pid_wait_until(15_in); //wait until 10 inches away from target to switch to ballgoal speed
  // //chassis.pid_speed_max_set(60);
  // chassis.pid_wait();

  // chassis.pid_drive_set(-9_in, DRIVE_SPEED); //drive to collect balls fallen balls 
  // chassis.pid_wait();
  // chassis.pid_drive_set(3_in, DRIVE_SPEED); //drive back to be in line with middle goal
  // chassis.pid_wait();
  // //chassis.pid_drive_set(-20_in, 69); //drive back to be in line with middle goal
  // //chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, TURN_SPEED); //220
  // chassis.pid_wait();
  // chassis.pid_drive_set(-6_in, 40); //drive back 20 inches to be fully in the parking zone
  // chassis.pid_wait();
  // middleIntakeMotor.move(127);
  // topIntakeMotor.move(-10);
  // chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  // chassis.pid_wait();
  // middleIntakeMotor.move(-20);
  // topIntakeMotor.move(-127);
  // chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  // chassis.pid_wait();
  // lilRaaahh.set(true);
  // //intake.move(-40);
  // chassis.pid_drive_set(-17_in, DRIVE_SPEED); //drive back 23
  // chassis.pid_wait_until(-15_in);
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // middleGoalScore.set(true);
  // chassis.pid_wait();
  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(-70); //-60 top 
  // chassis.pid_drive_set(-16_in,50); //towards goal 30 inches
  // chassis.pid_wait();
  // pros::delay(2400); //3500
  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(600); //3500
  // //chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  // //chassis.pid_wait();
  // //chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  // //chassis.pid_wait();
  // pros::delay(500); //3500

  //7 middle from before version
  // pros::delay(4000); //3500
  // chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // middleIntakeMotor.move(80); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(1000); //3500

  //park mid 7 new 
  chassis.pid_drive_set(21_in,DRIVE_SPEED); //7
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED); //40
  chassis.pid_wait();
  chassis.pid_drive_set(31_in,DRIVE_SPEED); //33 +10 and 42
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 85_deg, 120); //265 or 80
  chassis.pid_wait();
  chassis.pid_drive_set(60_in, 85); //drive forward 60 inches into the parking zone
  //chassis.pid_wait_until(15_in); //wait until 10 inches away from target to switch to ballgoal speed
  //chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED); //turn towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(-14, DRIVE_SPEED-40-20); //drive to collect balls fallen balls 
  chassis.pid_wait();
  chassis.pid_drive_set(2_in, DRIVE_SPEED); //drive back to be in line with middle goal
  chassis.pid_wait();
  //chassis.pid_drive_set(-20_in, 69); //drive back to be in line with middle goal
  //chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED); //220
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, 40); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(45_in, DRIVE_SPEED); //32
  chassis.pid_wait();
  middleIntakeMotor.move(-20);
  topIntakeMotor.move(-127);
  chassis.pid_turn_set(45_deg, TURN_SPEED); //220
  chassis.pid_wait();
  lilRaaahh.set(true);
  //intake.move(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED); //drive back 23
  chassis.pid_wait_until(-15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  middleGoalScore.set(true);
  chassis.pid_wait();
  // antiJam = new pros::Task(doIntakeUnstuck);
  middleIntakeMotor.move(110); //85 bottom 
  topIntakeMotor.move(-127); //-70 top 
  chassis.pid_drive_set(-20_in,50); //towards goal 30 inches
  chassis.pid_wait();
  pros::delay(3000); //3500
  // middleIntakeMotor.move(100); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(1000); //3500
  //chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  //chassis.pid_wait();
  //chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  //chassis.pid_wait();

  //7 middle from before version
  // pros::delay(4000); //3500
  // chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  // chassis.pid_wait();
  // middleIntakeMotor.move(80); //90 bottom 
  // topIntakeMotor.move(0); //-60 top 
  // pros::delay(1000); //3500
  //middleIntakeMotor.move(-50); //run the intake to score any remaining balls in the goal
  //pros::delay(1000); //3500
  //middleIntakeMotor.move(127); //stop the intake from spinning
  //pros::delay(1000);
  // chassis.pid_turn_set(30_deg, TURN_SPEED); //turn towards field perimeter wall
  // pros::delay(400);
  // chassis.pid_turn_set(60_deg, TURN_SPEED); //turn towards matchloader
  // pros::delay(400);
  // chassis.pid_turn_set(45_deg, TURN_SPEED); //turn towards field perimeter wall
  // chassis.pid_wait();
  //chassis.pid_drive_set(2_in, DRIVE_SPEED); //drive forward 20 inches to be fully in the parking zone
  //chassis.pid_wait();
  //middleGoalScore.set(false); //retract middle goal scoring piston
  // chassis.pid_drive_set(3_in, DRIVE_SPEED); //drive forward 50 inches into the parking zone
  // pros::delay(300);
  // chassis.pid_drive_set(-4_in, DRIVE_SPEED); //drive back 50 inches into the parking zone
  // chassis.pid_wait();
  
  // pros::delay(1500);

  // antiJam->remove();
  // delete antiJam;
  // antiJam = nullptr;
  //intake.move(127); //20
  lilRaaahh.set(false);

  //old 81 path stuff 
  chassis.pid_drive_set(57_in,DRIVE_SPEED); //move toward field perimeter by 55 inches 
  chassis.pid_wait();

  intake.move(0);
  middleIntakeMotor.move(127);
  lilRaaahh.set(true);
  chassis.pid_turn_set(0_deg, TURN_SPEED+10); //face the matchloader 
  chassis.pid_wait();
  middleGoalScore.set(false);
  middleIntakeMotor.move(127); //run bottom intake motor to intake the 3 balls
  topIntakeMotor.move(-15);
  pros::delay(100);

  chassis.pid_drive_set(27_in, DRIVE_SPEED); //30
  chassis.pid_wait_until(5_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  pros::delay(700); //delay for intaking at matchload 700 ms previously with in and out
  //   for (int i = 0; i < 3; i++) {
  //   chassis.pid_drive_set(2_in, BALLGOAL_SPEED+10); //back 1 in from matchload
  //   pros::delay(150);
  //   chassis.pid_turn_set(-3.5_deg, TURN_SPEED);
  //   pros::delay(100);
  //   chassis.pid_turn_set(3.5_deg, TURN_SPEED);
  //   pros::delay(100);
  // }
  pros::delay(600); //time for matchloading 700 with drive in out
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //matchload in and out 
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();

  //pros::delay(600); //previously 600 ms when in and out 

  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //backward to goal 15 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract
  chassis.pid_turn_set(135_deg,TURN_SPEED); // 135 deg to perimeter wall
  pros::delay(800); //1000
  intake.move(0); //intake stop spinning

  chassis.pid_drive_set(18_in,DRIVE_SPEED+5); //22 in towards perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(60_in,DRIVE_SPEED-10); //go towards other side 65 in
  chassis.pid_wait();
  chassis.pid_turn_set(230_deg,TURN_SPEED+10); //turn 130 before now 230 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(16_in,DRIVE_SPEED+5); //16 inches diagonal movement to long goal
  chassis.pid_wait();
  middleIntakeMotor.move(-10);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-13_in,DRIVE_SPEED+5); //backward to long goal by 22 in skillsCut
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1900); //delay for scoring at long goal 
  intake.move(0);

  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-20);
  pros::delay(100);
  chassis.pid_drive_set(41_in,DRIVE_SPEED); //forward 47 in skillsCut towards matchloader 
  chassis.pid_wait_until(15_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5); //+2
  pros::delay(1600); //delay for intaking at matchload 1200 ms previously with in and out
  //   for (int i = 0; i < 3; i++) {
  //   chassis.pid_drive_set(2_in, BALLGOAL_SPEED-30); //back 1 in from matchload
  //   pros::delay(150);
  //   chassis.pid_turn_set(173.5_deg, TURN_SPEED);
  //   pros::delay(100);
  //   chassis.pid_turn_set(183.5_deg, TURN_SPEED);
  //   pros::delay(100);
  // }
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(700); //time for matchloading 700 with drive in out
  //matchload in and out 
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();


  //intake.move(-10); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-35_in,DRIVE_SPEED+5+5); //backwards 33 before in skillsCut/now 37 to long goal 
  pros::delay(600);
  middleIntakeMotor.move(-10);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(100);
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //300 deg to perimeter wall
  pros::delay(2300); //delay for scoring at long goal

  //chassis.pid_drive_set(4_in,127); //back forth goals for control 4 out 
  //pros::delay(450);
  //chassis.pid_drive_set(-5_in, 80); //back forth goals for control 5 in
  //pros::delay(450);
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //new park
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait();
  chassis.pid_turn_set(220_deg, TURN_SPEED); //228
  chassis.pid_wait();
  chassis.pid_drive_set(35_in,DRIVE_SPEED+10); //33 +10 and 42
  pros::delay(1100);
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120); //265
  pros::delay(600);
  chassis.pid_drive_set(32_in, 85 ); //125 and 28
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 85); //125 and 35
  chassis.pid_wait();

}

// Global variable to track when intake became stuck
uint32_t intakeStuckTime = 0;

void doIntakeUnstuck() {
    // Check if intake is stuck: low velocity but high voltage
    if (fabs(intake.get_actual_velocity()) < 2 && fabs(intake.get_voltage()) > 2000) {
        if (intakeStuckTime == 0) {
            // Start timing how long intake has been stuck
            intakeStuckTime = pros::millis();
        } else if (pros::millis() - intakeStuckTime > 400) {
            // Intake has been stuck for 400ms, reverse it briefly
            double lowerIntakePower = middleIntakeMotor.get_power();  // Store current power
            double upperIntakePower = topIntakeMotor.get_power();  // Store current power
            intake.move(-20);  // Slight reverse to unstick
            pros::delay(250);  // Hold reverse for 250ms
            middleIntakeMotor.move(lowerIntakePower);  // Return to original power
            topIntakeMotor.move(upperIntakePower);  // Return to original power
            intakeStuckTime = 0;  // Reset timer
        } 
    } else {
        // Intake is not stuck anymore, reset timer
        intakeStuckTime = 0;
    }
}

void passiveDrive(){
  middleIntakeMotor.move(90); //90 bottom 
  topIntakeMotor.move(-70); //-60 top 
  middleGoalScore.set(true);
  chassis.pid_drive_set(-16_in,50); //towards goal 30 inches
  chassis.pid_wait();
}
void frontParkClear() {
  chassis.drive_angle_set(180_deg); //face forward
  chassis.pid_wait();
  middleIntakeMotor.move(127); //bottom stage intake spins
  topIntakeMotor.move(-20); //top stage intake spins at reduced speed
  chassis.pid_drive_set(40_in, DRIVE_SPEED-20-20-20); //drive 60 inches forward into the parking zone
  chassis.pid_wait();
  for (int i = 0; i < 4; i++) {
    chassis.pid_drive_set(4_in, BALLGOAL_SPEED-40); //back 1 in from matchload
    pros::delay(300);
    chassis.pid_turn_set(175.5_deg, TURN_SPEED);
    pros::delay(300);
    chassis.pid_turn_set(185.5_deg, TURN_SPEED);
    pros::delay(300);
  }
  chassis.pid_drive_set(-20_in, DRIVE_SPEED); //drive back 20 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //turn towards field perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED-60); //drive forward 20 inches to be in line with middle goal
  chassis.pid_wait();
}

void fivePlusThree(){
  descore.set(true); //make sure descore is retracted at beginning of match
  chassis.drive_angle_set(-24.5_deg);  
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(30_in, DRIVE_SPEED); //32
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  lilRaaahh.set(true);

  chassis.pid_wait();
  //pros::delay(50);

  chassis.pid_turn_set(240_deg, TURN_SPEED);   
  chassis.pid_wait();
  chassis.pid_drive_set(31_in,DRIVE_SPEED-8-5); //32.5 non issam  and ballgoal + 10
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  lilRaaahh.set(true);
  middleIntakeMotor.move(30);
  chassis.pid_drive_set(-12_in,DRIVE_SPEED); //drive back into the matchloader by 11 inches
  pros::delay(500);
  intake.move(127); //start intake
  pros::delay(1210); //score the 4 balls 1300 usual
  middleIntakeMotor.move(127);
  topIntakeMotor.move(0);

  chassis.pid_drive_set(37_in,DRIVE_SPEED); //35
  chassis.pid_wait_until(14_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5-5);

   //forward 37 towards matchloader 
  pros::delay(1100);

   //forward 37 towards matchloader 
  //intake.move(-15); //20
  // middleIntakeMotor.move(35);

  chassis.pid_drive_set(-12,DRIVE_SPEED); //-11.5
  chassis.pid_wait();
  //lilRaaahh.set(false);
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  lilRaaahh.set(false);

  middleIntakeMotor.move(127);
  topIntakeMotor.move(-15);
  // topIntakeMotor.move(-20);
  chassis.pid_drive_set(-42_in, DRIVE_SPEED); //55
  chassis.pid_wait();

  chassis.pid_turn_set(285_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(17_in, DRIVE_SPEED+10); //drive in by 16 incehs into matchloader 18
  chassis.pid_wait_until(2_in); //wait for 4 inches before switching to ballgoal speed 
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20); 
  chassis.pid_wait_until(12_in); //wait for 4 inches before switching to ballgoal speed
  lilRaaahh.set(true); //deploy matchload mech once at cluster of balls
  chassis.pid_wait();
  pros::delay(20);

  chassis.pid_drive_set(-19_in, DRIVE_SPEED+10); //drive in by 16 incehs into matchloader 18
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED+5); //drive 33 inches toward 3 cluster of balls at drive_speed-5
  chassis.pid_wait();

  chassis.pid_turn_set(226_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-13_in, DRIVE_SPEED); //55
  chassis.pid_wait_until(-5_in);
  chassis.pid_speed_max_set(BALLGOAL_SPEED);
  chassis.pid_wait();

  lilRaaahh.set(true);
  intake.move(-30);
  middleGoalScore.set(true);
  middleIntakeMotor.move(100);  //110
  topIntakeMotor.move(-60);
}

void inverse34(){
  chassis.drive_angle_set(-24.5_deg);  //set initial angle to -24.5 deg 
  chassis.pid_wait();
  middleIntakeMotor.move(127); //start bottom intake motor to intake balls
  topIntakeMotor.move(-20); //passive outake back
  chassis.pid_drive_set(30_in, DRIVE_SPEED); //drive 30 inches forward into 3 ball stack 
  chassis.pid_wait_until(15_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20+20);
  lilRaaahh.set(true); //deploy matchloader to secure 3 balls
  chassis.pid_wait();
  pros::delay(50); //delay to ensure ball picked up 
  chassis.pid_turn_set(240_deg, TURN_SPEED); //turn towards matchloader and field perimeter wall
  pros::delay(700); //pros delay for turning 
  chassis.pid_drive_set(29.5_in,DRIVE_SPEED-8-5); //drive 29.5 inches towards matchloader and field perimeter wall 
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //face towards matchloader
  chassis.pid_wait();
  middleIntakeMotor.move(30); //start pushing balls upwards in intake 
  chassis.pid_drive_set(-12_in,DRIVE_SPEED); //drive back into the goal by 12 inches
  pros::delay(500); //delay for driving back into goal 
  intake.move(127); //start intake
  pros::delay(1300); //score the 4 balls 1300 ms
  middleIntakeMotor.move(127); //start bottom intake motor
  topIntakeMotor.move(-20); //passive top outake backwards
  chassis.pid_drive_set(36_in,DRIVE_SPEED); //drive 36 inches towards matchloader
  chassis.pid_wait_until(14_in); //at 14 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED-5-5);
  pros::delay(940); //intake three balls 940 ms
  chassis.pid_drive_set(-9.5_in,DRIVE_SPEED); //drive back 9.5 inches 
  chassis.pid_wait();
  chassis.pid_turn_set(226_deg, TURN_SPEED); //face middle goal backwards
  chassis.pid_wait();
  middleIntakeMotor.move(-10); //outake slightly to prevent jamming at middle goal 
  topIntakeMotor.move(-40);
  chassis.pid_drive_set(-56_in, DRIVE_SPEED-5); //drive back 56 inches towards middle goal
  chassis.pid_wait_until(-40_in); //at -40 inches switch to ballgoal speed
  chassis.pid_speed_max_set(BALLGOAL_SPEED);
  chassis.pid_wait_until(-25_in); //at -25 inches deploy middle goal scoring piston
  middleGoalScore.set(true); 
  chassis.pid_wait();
  middleIntakeMotor.move(100);  //start scoring in middle goal 
  topIntakeMotor.move(-60);
  pros::delay(1800); //scoring time for 3 balls 1800 ms
  chassis.pid_drive_set(14_in,DRIVE_SPEED); //move forward out of middle goal 
  chassis.pid_wait();
  middleGoalDescore.set(true); //activate middle goal descore 
  chassis.pid_drive_set(-11_in,DRIVE_SPEED); //drive back into middle goal and descore opponent balls 
  chassis.pid_wait();
  chassis.pid_drive_set(36_in,DRIVE_SPEED); //drive forward towards goal
  chassis.pid_wait();
  middleGoalDescore.set(false); //retract middle goal descore
  middleGoalScore.set(false);  //retract middle goal score+
  chassis.pid_turn_set(180_deg, TURN_SPEED); //stay parallel with the goal 
  chassis.pid_wait();
  descore.set(false); //activate descore 
  chassis.pid_drive_set(-27_in,DRIVE_SPEED-5); //+5 and -27 for fast -10 slow and -25 - used to drive back into goal for descore
  chassis.pid_wait();
  lilRaaahh.set(false); //pull up matchloader
}

void ogSkills(){
  //first quadrant 
  chassis.drive_angle_set(90_deg); //face 90 deg to perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(32_in,BALLGOAL_SPEED+15); //towards goal 30 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //lil will deploy
  pros::delay(100); //delays for consistency 
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchloader
  chassis.pid_wait();
  // lilRaaahh.set(true); //matchloader deploy
  middleIntakeMotor.move(127); //first stage intake spin 
  topIntakeMotor.move(-10);
  pros::delay(100); //pause until lil will and first stage engaged 
  chassis.pid_drive_set(21,BALLGOAL_SPEED+10); //towards matchload 20 inches
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(1150); //time for matchloading 1100
  middleIntakeMotor.move(0); //stop intake bottom stage 
  chassis.pid_drive_set(-17_in, BALLGOAL_SPEED+10); //backward to goal 17 inches
  chassis.pid_wait();
  pros::delay(100);
  lilRaaahh.set(false); //matchloader retract
  intake.move(0); //intake stop spinning
  pros::delay(100);
  chassis.pid_turn_set(45_deg,TURN_SPEED); //45 deg to perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(15_in,BALLGOAL_SPEED); //19 in towards perimeter wall
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(0_deg,TURN_SPEED); //turn towards other side matchload/perimeter 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(65_in,DRIVE_SPEED-30); //go towards other side 65 in
  chassis.pid_wait();
  pros::delay(100);

  //next quadrant
  chassis.pid_turn_set(310_deg,TURN_SPEED); //turn 310 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(16_in,DRIVE_SPEED-20); //16 inches diagonal movement to long goal
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(0_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in,DRIVE_SPEED-20); //backward to long goal by 22 in
  chassis.pid_wait();
  intake.move(127); //intaking at goals
  pros::delay(2000); //delay for scoring at long goal 
  intake.move(0);
  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-10); 
  chassis.pid_drive_set(37_in,BALLGOAL_SPEED+10); //forward 37 towards matchloader 
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in, BALLGOAL_SPEED+10); //1 inches back from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in, BALLGOAL_SPEED+10); //2 inches forward into matchload
  chassis.pid_wait();
  pros::delay(150); //matchloader timing 
  intake.move(-10); //moving intake backward to prevent jamming
  pros::delay(600); //delay for intaking at matchload 
  chassis.pid_drive_set(-30_in,DRIVE_SPEED-20); //backwards 34 to long goal 
  chassis.pid_wait();
  pros::delay(100); 
  // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // topIntakeMotor.move(100);
  intake.move(127);
  pros::delay(2000); //delay for scoring at long goal
  chassis.pid_drive_set(5_in,127); //back forth goals for control 5 out 
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in,30); //back forth goals for control 6 in
  chassis.pid_wait();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //other section side 
  //3rd quadrant 
  chassis.pid_drive_set(15_in,DRIVE_SPEED); //long goal to middle tile by 15 in
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(270_deg,TURN_SPEED); //turn 270 to left facing other side perimeter  
  chassis.pid_wait();
  chassis.pid_drive_set(96_in,DRIVE_SPEED-40); //forward by 94 in to other side 
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(0_deg,TURN_SPEED); //turn towards matchloader
  chassis.pid_wait();
  middleIntakeMotor.move(127); //start intake for matchload
  topIntakeMotor.move(-10);
  lilRaaahh.set(true); //deploy lil will for matchload
  chassis.pid_drive_set(27_in,BALLGOAL_SPEED+10); //forward 27 in towards matchloader
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in,BALLGOAL_SPEED+10); //back 2 in from matchload -2 and drivespeed 
  chassis.pid_wait();
  chassis.pid_drive_set(3_in, BALLGOAL_SPEED+10); //forward 3 in to matchload 3 and drivespeed
  chassis.pid_wait();
  //intake.move(-10); outaking to prevent jamming 
  pros::delay(1200); //time for matchloading
  chassis.pid_drive_set(-18_in,DRIVE_SPEED); //backwards 18 in
  chassis.pid_wait();
  pros::delay(100);
  intake.move(-5); //outaking to prevent jamming
  lilRaaahh.set(false); //lil will retract
  middleIntakeMotor.move(0); //stop intake
  pros::delay(100);
  chassis.pid_turn_set(320_deg,TURN_SPEED); //320 deg turn to perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(20_in,DRIVE_SPEED); //21 towards perimeter wall 
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(183_deg,TURN_SPEED); //turn towards other side perimeter/matchload
  chassis.pid_wait();
  chassis.pid_drive_set(84_in,DRIVE_SPEED-10); // go towards matchload 84 in
  chassis.pid_wait();
  pros::delay(100);

  //final (4th) quadrant
  chassis.pid_turn_set(135_deg,TURN_SPEED); //first angle turn towards goal 135 deg
  chassis.pid_wait();
  chassis.pid_drive_set(17.5_in,DRIVE_SPEED); //first movement to matchload diagonal 17.5 before
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(183_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 183
  chassis.pid_wait();
  chassis.pid_drive_set(-14_in,DRIVE_SPEED-20); //backward to long goal by 16 in
  chassis.pid_wait();
  //pros::delay(100); 
  intake.move(127); //intake start 
  pros::delay(2000); //time for scoring at long goal
  intake.move(0); //intake stop
  lilRaaahh.set(true); //lil will deploy
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(38_in,BALLGOAL_SPEED+10); //forward 37 towards matchloade ballgoal+10
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload 
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload 
  chassis.pid_wait();
  pros::delay(100); //delay for consistency 
  intake.move(-10);
  pros::delay(450); //time for matchloading 400 ms
  chassis.pid_drive_set(-31_in,DRIVE_SPEED-20); //backwards 30 to long goal
  chassis.pid_wait();
  //pros::delay(100);
  intake.move(127); //start intake
  pros::delay(2000); //time for scoring at long goal 
  chassis.pid_drive_set(5_in,60); //forwrad 5 goals for control 
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in,30); //bckward 6 goals for control 
  chassis.pid_wait();
  lilRaaahh.set(false);
  intake.move(0); //stop intake

  //new park
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait();
  chassis.pid_turn_set(140_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(37_in,DRIVE_SPEED+10); //33
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 95_deg, 120);
  chassis.pid_wait();
  // chassis.pid_drive_set(35_in, 125); //125 and 35
  // chassis.pid_wait();
  chassis.pid_drive_set(29_in, 90); //125 and 28
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 90); //125 and 35
  chassis.pid_wait();
}

void fast81(){
  //first quadrant 
  descore.set(true);
  chassis.drive_angle_set(270_deg); //face 270 deg to perimeter wall
  chassis.pid_wait_quick();
  chassis.pid_drive_set(31.5_in,DRIVE_SPEED); //towards goal 33 inches
  chassis.pid_wait_quick();
  lilRaaahh.set(true); //lil will deploy
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //turn towards matchloader
  chassis.pid_wait_quick();
  pros::delay(100);
  // lilRaaahh.set(true); //matchloader deploy
  middleIntakeMotor.move(127); //first stage intake spin 
  topIntakeMotor.move(-15);
  chassis.pid_drive_set(12_in,DRIVE_SPEED-10); //towards matchload 15 inches
  chassis.pid_wait_until(6_in); //at 6 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait_quick();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait_quick();
  pros::delay(700); //time for matchloading 
  chassis.pid_drive_set(-17_in, DRIVE_SPEED+10); //backward to goal 17 inches
  chassis.pid_wait_quick();
  lilRaaahh.set(false); //matchloader retract

  // //turning to other side/going to other side
  chassis.pid_turn_set(300_deg,TURN_SPEED+10); //300 deg to perimeter wall
  pros::delay(1000); //1000
  intake.move(0); //intake stop spinning
  chassis.pid_drive_set(16_in, DRIVE_SPEED+10); //16 in towards perimeter wall
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 0 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(64_in,DRIVE_SPEED-10-5-5); //go towards other side 65 in
  chassis.pid_wait_quick();

  // //next quadrant
  chassis.pid_turn_set(53_deg,TURN_SPEED+10); //turn 50 deg to start alinging with long goal
  chassis.pid_wait_quick();
  chassis.pid_drive_set(15_in,DRIVE_SPEED+10); //17 inches diagonal movement to long goal
  pros::delay(650);
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards matchload with bot facing forwards 0 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-14_in,DRIVE_SPEED+10); //backward to long goal by 15
  // chassis.pid_wait();
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1950); //delay for scoring at long goal 
  intake.move(0);
  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-15);
  pros::delay(100);
  chassis.pid_drive_set(42_in,DRIVE_SPEED); //forward 43
  chassis.pid_wait_until(12_in); //at 12 inches switch to ballgoalspeed+10
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait_quick();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait_quick();
  pros::delay(600); //delay for intaking at matchload 600 ms
  intake.move(-5); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-33_in,DRIVE_SPEED+10); //backwards 33 
  chassis.pid_wait_quick();
  //pros::delay(300);
  // // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // // topIntakeMotor.move(100);
  // intake.move(127); //intake 
  middleIntakeMotor.move(127);
  topIntakeMotor.move(100);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127);//back forth goals for control 4 out 
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  chassis.pid_wait_quick();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //middle section 
  //first 4 balls stack
  // chassis.pid_drive_set(20_in, DRIVE_SPEED); //drive towards long goal 23 inches 
  // chassis.pid_wait();
  // chassis.pid_turn_set(135_deg,TURN_SPEED+5); //135 deg 
  // pros::delay(850); //850 delay ms 
  // middleIntakeMotor.move(127);
  // chassis.pid_drive_set(40_in, DRIVE_SPEED+10); //drive towards cluster of balls 40 inches
  // chassis.pid_wait_until(13_in); //at 14 inches switch to ballgoalspeed -20 
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // chassis.pid_wait();

  chassis.pid_drive_set(22_in, DRIVE_SPEED); //drive towards long goal
  chassis.pid_wait_quick();
  chassis.pid_turn_set(134_deg,TURN_SPEED); //224
  chassis.pid_wait_quick();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(40_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(12_in);//at 35 inches switch to ballgoalspeed -15
  chassis.pid_speed_max_set(BALLGOAL_SPEED-30);
  chassis.pid_wait_quick();

  //second 4 balls stack
  chassis.pid_turn_set(86_deg, TURN_SPEED+10); //turn toward other 4 balls stack 86 before
  chassis.pid_wait_quick();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(53_in, DRIVE_SPEED+5); //drive towards long goal 53 old 
  chassis.pid_wait_until(21_in);//at 21 inches switch to ballgoalspeed -20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait_quick();
  topIntakeMotor.move(-20);
  middleIntakeMotor.move(0);
  
  chassis.pid_drive_set(-9_in, DRIVE_SPEED+10); //back away from balls a bit 10 inches 
  chassis.pid_wait_quick();
  lilRaaahh.set(true); //prepare matchloader to score
  //middle goal scoring 
  chassis.pid_turn_set(45_deg,TURN_SPEED+10); //45
  //lilRaaahh.set(true);
  chassis.pid_wait_quick();
  intake.move(-40);
  //middleIntakeMotor.move(-30);
  //topIntakeMotor.move(-60);
  chassis.pid_drive_set(-22_in, DRIVE_SPEED); //drive back 23
  chassis.pid_wait_until(-19_in);
  middleGoalScore.set(true);
  chassis.pid_wait_quick();
  //intake.move(0);
  // middleIntakeMotor.move(110);
  // //topIntakeMotor.move(-40);
  // //topIntakeMotor.move(-80); //old middle goal scoring 
  // pros::delay(2200); //delay for scoring at middle goal
  // middleIntakeMotor.move(90);
  // pros::delay(1500); //delay for scoring at middle goal

  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(-70); //-60 top 
  // pros::delay(3500); //3500

  middleIntakeMotor.move(90); //90 bottom 
  topIntakeMotor.move(-70); //-60 top 
  pros::delay(2900); //3500
  middleIntakeMotor.move(80); //90 bottom 
  topIntakeMotor.move(0); //-60 top 
  pros::delay(600); //3500
  chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  chassis.pid_wait_quick();
  pros::delay(500);


  // topIntakeMotor.move(90);
  // middleIntakeMotor.move(-40);
  intake.move(127); //20
  lilRaaahh.set(false);

  //new stuff for skills
  chassis.pid_drive_set(53_in,DRIVE_SPEED+5+5); //move toward field perimeter by 55 inches 
  chassis.pid_wait_quick();
  // middleGoalScore.set(false);

  intake.move(0);
  middleIntakeMotor.move(127);
  lilRaaahh.set(true);
  chassis.pid_turn_set(0_deg, TURN_SPEED+10); //face the matchloader 
  chassis.pid_wait_quick();
  pros::delay(100);
  middleGoalScore.set(false);
  // lilRaaahh.set(true);
  middleIntakeMotor.move(127); //run bottom intake motor to intake the 3 balls
  topIntakeMotor.move(-15);
  //topIntakeMotor.move(0); //stop top intake motor
  chassis.pid_drive_set(29_in, BALLGOAL_SPEED); //30
  chassis.pid_wait_until(5_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait_quick();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait_quick();
  pros::delay(600);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //backward to goal 15 inches
  chassis.pid_wait_quick();
  lilRaaahh.set(false); //matchloader retract
  chassis.pid_turn_set(135_deg,TURN_SPEED); // 135 deg to perimeter wall
  chassis.pid_wait_quick();
  intake.move(0); //intake stop spinning

  chassis.pid_drive_set(18_in,DRIVE_SPEED+5); //22 in towards perimeter wall
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 180 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(60_in,DRIVE_SPEED-10-5); //go towards other side 65 in
  chassis.pid_wait_quick();
  chassis.pid_turn_set(230_deg,TURN_SPEED+10); //turn 130 before now 230 deg to start alinging with long goal
  chassis.pid_wait_quick();
  chassis.pid_drive_set(17_in,DRIVE_SPEED+5); //16 inches diagonal movement to long goal
  chassis.pid_wait_quick();
  middleIntakeMotor.move(-10);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 180 deg
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-13_in,DRIVE_SPEED+5); //backward to long goal by 22 in skillsCut
  // chassis.pid_wait();
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1900); //delay for scoring at long goal 
  intake.move(0);

  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-30);
  pros::delay(100);
  chassis.pid_drive_set(41_in,DRIVE_SPEED); //forward 47 in skillsCut towards matchloader 
  chassis.pid_wait_until(15_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5); //+2
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait_quick();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait_quick();
  pros::delay(450); //delay for intaking at matchload 450 ms

  //intake.move(-10); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-35_in,DRIVE_SPEED+5+5); //backwards 33 before in skillsCut/now 37 to long goal 
  pros::delay(700);
  middleIntakeMotor.move(-10);
  // // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // // topIntakeMotor.move(100);
  //intake.move(127);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(127);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127); //back forth goals for control 4 out 
  pros::delay(450);
  chassis.pid_drive_set(-5_in, 80); //back forth goals for control 5 in
  pros::delay(450);
  // chassis.pid_drive_set(4_in,DRIVE_SPEED); //back forth goals for control 4 out 
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  // chassis.pid_wait();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //new park
  // chassis.pid_drive_set(8_in,DRIVE_SPEED+10); //8
  // chassis.pid_wait();
  // chassis.pid_turn_set(228_deg, TURN_SPEED+20);
  // chassis.pid_wait();
  // chassis.pid_drive_set(42_in,DRIVE_SPEED+10); //33
  // pros::delay(900);

  // middleIntakeMotor.move(127);
  // //topIntakeMotor.move(-40);
  // chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120);
  // // chassis.pid_wait();
  // pros::delay(300);
  // chassis.pid_drive_set(31_in, 125); //125 and 28
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 125); //125 and 35
  // chassis.pid_wait();

  //old pre tune
  // chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  // chassis.pid_wait();
  // chassis.pid_turn_set(228_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(42_in,DRIVE_SPEED-10); //33 +10
  // chassis.pid_wait();
  // //pros::delay(900);
  // middleIntakeMotor.move(127);
  // chassis.pid_swing_set(ez::LEFT_SWING, 260_deg, 120); //265
  // chassis.pid_wait();
  // // pros::delay(300);
  // // chassis.pid_drive_set(35_in, 125); //125 and 35
  // // chassis.pid_wait();
  // chassis.pid_drive_set(29_in, 85); //125 and 28
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 85); //125 and 35
  // chassis.pid_wait();

  chassis.drive_angle_set(180_deg);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait_quick();
  chassis.pid_turn_set(220_deg, TURN_SPEED); //228
  chassis.pid_wait_quick();
  chassis.pid_drive_set(35_in,DRIVE_SPEED+10); //33 +10 and 42
  //chassis.pid_wait();
  pros::delay(1100);
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120); //265
  //chassis.pid_wait();
  pros::delay(600);
  // chassis.pid_drive_set(35_in, 125); //125 and 35
  // chassis.pid_wait();
  chassis.pid_drive_set(32_in, 85 ); //125 and 28
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-4_in, 85); //125 and 35
  chassis.pid_wait_quick();
}

void leftSideSkills(){
  //first quadrant 
  descore.set(true);
  chassis.drive_angle_set(270_deg); //face 270 deg to perimeter wall
  chassis.pid_wait();
  chassis.pid_drive_set(31.5_in,DRIVE_SPEED); //towards goal 33 inches
  chassis.pid_wait();
  lilRaaahh.set(true); //lil will deploy
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //turn towards matchloader
  chassis.pid_wait();
  pros::delay(100);
  // lilRaaahh.set(true); //matchloader deploy
  middleIntakeMotor.move(127); //first stage intake spin 
  topIntakeMotor.move(-35);
  chassis.pid_drive_set(11_in,DRIVE_SPEED-14); //towards matchload 15 inches
  chassis.pid_wait_until(6_in); //at 6 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED-13); //-10
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(700); //time for matchloading 
  chassis.pid_drive_set(-17_in, DRIVE_SPEED+10); //backward to goal 17 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract

  // //turning to other side/going to other side
  chassis.pid_turn_set(300_deg,TURN_SPEED+10); //300 deg to perimeter wall
  pros::delay(1000); //1000
  intake.move(0); //intake stop spinning
  chassis.pid_drive_set(16_in, DRIVE_SPEED+10); //16 in towards perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(64_in,DRIVE_SPEED-10-5-5); //go towards other side 65 in
  chassis.pid_wait();

  // //next quadrant
  chassis.pid_turn_set(53_deg,TURN_SPEED+10); //turn 50 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(14.5_in,DRIVE_SPEED+10); //17 inches diagonal movement to long goal
  pros::delay(650);
  chassis.pid_turn_set(0_deg,TURN_SPEED+10); //turn towards matchload with bot facing forwards 0 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-14_in,DRIVE_SPEED+10); //backward to long goal by 15
  // chassis.pid_wait();
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1950); //delay for scoring at long goal 
  intake.move(0);
  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-15);
  pros::delay(100);
  chassis.pid_drive_set(43_in,DRIVE_SPEED); //forward 43
  chassis.pid_wait_until(12_in); //at 12 inches switch to ballgoalspeed+10
  chassis.pid_speed_max_set(BALLGOAL_SPEED); //ballgoal
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(600); //delay for intaking at matchload 600 ms
  intake.move(-5); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-32_in,DRIVE_SPEED+5); //backwards 33 
  chassis.pid_wait();
  //pros::delay(300);
  // // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // // topIntakeMotor.move(100);
  // intake.move(127); //intake 
  middleIntakeMotor.move(127);
  topIntakeMotor.move(127);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127);//back forth goals for control 4 out 
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  chassis.pid_wait();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //middle section 
  //first 4 balls stack
  // chassis.pid_drive_set(20_in, DRIVE_SPEED); //drive towards long goal 23 inches 
  // chassis.pid_wait();
  // chassis.pid_turn_set(135_deg,TURN_SPEED+5); //135 deg 
  // pros::delay(850); //850 delay ms 
  // middleIntakeMotor.move(127);
  // chassis.pid_drive_set(40_in, DRIVE_SPEED+10); //drive towards cluster of balls 40 inches
  // chassis.pid_wait_until(13_in); //at 14 inches switch to ballgoalspeed -20 
  // chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  // chassis.pid_wait();

  chassis.pid_drive_set(22_in, DRIVE_SPEED); //drive towards long goal
  chassis.pid_wait();
  chassis.pid_turn_set(134_deg,TURN_SPEED); //224
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(40_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(12_in);//at 35 inches switch to ballgoalspeed -15
  chassis.pid_speed_max_set(BALLGOAL_SPEED-30);
  chassis.pid_wait();

  //second 4 balls stack
  chassis.pid_turn_set(86_deg, TURN_SPEED+10); //turn toward other 4 balls stack 86 before
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(53_in, DRIVE_SPEED+5); //drive towards long goal 53 old 
  chassis.pid_wait_until(21_in);//at 21 inches switch to ballgoalspeed -20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait();
  topIntakeMotor.move(-20);
  middleIntakeMotor.move(0);
  
  chassis.pid_drive_set(-11_in, DRIVE_SPEED+10); //back away from balls a bit 10 inches 
  chassis.pid_wait();
  lilRaaahh.set(true); //prepare matchloader to score
  //middle goal scoring 
  chassis.pid_turn_set(45_deg,TURN_SPEED+10); //45
  //lilRaaahh.set(true);
  chassis.pid_wait();
  intake.move(-35); //intake back
  // topIntakeMotor.move(-127);
  // middleIntakeMotor.move(-20);
  //middleIntakeMotor.move(-30);
  //topIntakeMotor.move(-60);
  chassis.pid_drive_set(-22_in, DRIVE_SPEED); //drive back 23
  chassis.pid_wait_until(-19_in);
  middleGoalScore.set(true);
  chassis.pid_wait();
  //intake.move(0);
  // middleIntakeMotor.move(110);
  // //topIntakeMotor.move(-40);
  // //topIntakeMotor.move(-80); //old middle goal scoring 
  // pros::delay(2200); //delay for scoring at middle goal
  // middleIntakeMotor.move(90);
  // pros::delay(1500); //delay for scoring at middle goal

  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(-70); //-60 top 
  // pros::delay(3500); //3500

  middleIntakeMotor.move(100); //90 bottom  
  topIntakeMotor.move(-60); //-60 top 
  pros::delay(2900); //3500
  //middleIntakeMotor.move(80); //90 bottom 
  //topIntakeMotor.move(0); //-60 top 
  //pros::delay(1100); //3500
  chassis.pid_drive_set(4_in, DRIVE_SPEED-50); //drive forward 23 inches to be fully in the parking zone
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED-50); //drive back 23 inches to be fully in the parking zone
  chassis.pid_wait();
  pros::delay(1200);


  // topIntakeMotor.move(90);
  // middleIntakeMotor.move(-40);
  intake.move(127); //20
  lilRaaahh.set(false);

  //new stuff for skills
  chassis.pid_drive_set(54_in,DRIVE_SPEED+5+5+5); //move toward field perimeter by 55 inches 
  chassis.pid_wait();
  // middleGoalScore.set(false);

  intake.move(0);
  middleIntakeMotor.move(127);
  lilRaaahh.set(true);
  chassis.pid_turn_set(0_deg, TURN_SPEED+10); //face the matchloader 
  chassis.pid_wait();
  pros::delay(100);
  middleGoalScore.set(false);
  // lilRaaahh.set(true);
  middleIntakeMotor.move(127); //run bottom intake motor to intake the 3 balls
  topIntakeMotor.move(-15);
  //topIntakeMotor.move(0); //stop top intake motor
  chassis.pid_drive_set(29_in, BALLGOAL_SPEED); //30
  chassis.pid_wait_until(5_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED+5-5-5);
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(600);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED+10); //backward to goal 15 inches
  chassis.pid_wait();
  lilRaaahh.set(false); //matchloader retract
  chassis.pid_turn_set(135_deg,TURN_SPEED); // 135 deg to perimeter wall
  // chassis.pid_wait();
  pros::delay(800);
  intake.move(0); //intake stop spinning

  chassis.pid_drive_set(18_in,DRIVE_SPEED+5); //22 in towards perimeter wall
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,TURN_SPEED+10); //turn towards other side matchload/perimeter 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(60_in,DRIVE_SPEED-10-5); //go towards other side 65 in
  chassis.pid_wait();
  chassis.pid_turn_set(230_deg,TURN_SPEED+10); //turn 130 before now 230 deg to start alinging with long goal
  chassis.pid_wait();
  chassis.pid_drive_set(16_in,DRIVE_SPEED+5); //16 inches diagonal movement to long goal
  chassis.pid_wait();
  middleIntakeMotor.move(-10);
  chassis.pid_turn_set(180_deg,TURN_SPEED); //turn towards matchload with bot facing forwards 180 deg
  chassis.pid_wait();
  chassis.pid_drive_set(-13_in,DRIVE_SPEED+5); //backward to long goal by 22 in skillsCut
  // chassis.pid_wait();
  pros::delay(300);
  intake.move(127); //intaking at goals
  pros::delay(1900); //delay for scoring at long goal 
  intake.move(0);

  lilRaaahh.set(true); //matchload mech deploy 
  middleIntakeMotor.move(127); //first stage intake spin
  topIntakeMotor.move(-30);
  pros::delay(100);
  chassis.pid_drive_set(41_in,DRIVE_SPEED); //forward 47 in skillsCut towards matchloader 
  chassis.pid_wait_until(15_in); //at 15 inches switch to ballgoalspeed
  chassis.pid_speed_max_set(BALLGOAL_SPEED-2); //+2
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in,BALLGOAL_SPEED+10); //back 1 in from matchload
  chassis.pid_wait();
  chassis.pid_drive_set(2_in,BALLGOAL_SPEED+10); //forward 2 in to matchload
  chassis.pid_wait();
  pros::delay(450); //delay for intaking at matchload 450 ms

  //intake.move(-10); //moving intake backward to prevent jamming
  chassis.pid_drive_set(-35_in,DRIVE_SPEED+5); //backwards 33 before in skillsCut/now 37 to long goal +10
  pros::delay(800);
  middleIntakeMotor.move(-10);
  // // middleIntakeMotor.move(127); //controlled intake scoring so balls dont bounce out
  // // topIntakeMotor.move(100);
  //intake.move(127);
  middleIntakeMotor.move(127);
  topIntakeMotor.move(127);
  pros::delay(2200); //delay for scoring at long goal
  chassis.pid_drive_set(4_in,127); //back forth goals for control 4 out 
  pros::delay(450);
  chassis.pid_drive_set(-5_in, 70); //back forth goals for control 5 in
  pros::delay(450);
  // chassis.pid_drive_set(4_in,DRIVE_SPEED); //back forth goals for control 4 out 
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 30); //back forth goals for control 5 in
  // chassis.pid_wait();
  middleIntakeMotor.move(0); //stop intake stages 
  topIntakeMotor.move(0);
  lilRaaahh.set(false); //retract matchloader mech

  //new park
  // chassis.pid_drive_set(8_in,DRIVE_SPEED+10); //8
  // chassis.pid_wait();
  // chassis.pid_turn_set(228_deg, TURN_SPEED+20);
  // chassis.pid_wait();
  // chassis.pid_drive_set(42_in,DRIVE_SPEED+10); //33
  // pros::delay(900);

  // middleIntakeMotor.move(127);
  // //topIntakeMotor.move(-40);
  // chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120);
  // // chassis.pid_wait();
  // pros::delay(300);
  // chassis.pid_drive_set(31_in, 125); //125 and 28
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 125); //125 and 35
  // chassis.pid_wait();

  //old pre tune
  // chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  // chassis.pid_wait();
  // chassis.pid_turn_set(228_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(42_in,DRIVE_SPEED-10); //33 +10
  // chassis.pid_wait();
  // //pros::delay(900);
  // middleIntakeMotor.move(127);
  // chassis.pid_swing_set(ez::LEFT_SWING, 260_deg, 120); //265
  // chassis.pid_wait();
  // // pros::delay(300);
  // // chassis.pid_drive_set(35_in, 125); //125 and 35
  // // chassis.pid_wait();
  // chassis.pid_drive_set(29_in, 85); //125 and 28
  // chassis.pid_wait();
  // chassis.pid_drive_set(-5_in, 85); //125 and 35
  // chassis.pid_wait();

  chassis.drive_angle_set(180_deg);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait();
  chassis.pid_turn_set(220_deg, TURN_SPEED); //228
  chassis.pid_wait();
  chassis.pid_drive_set(35_in,DRIVE_SPEED+10); //33 +10 and 42
  //chassis.pid_wait();
  pros::delay(1100);
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120); //265
  //chassis.pid_wait();
  pros::delay(600);
  // chassis.pid_drive_set(35_in, 125); //125 and 35
  // chassis.pid_wait();
  chassis.pid_drive_set(31_in, 70 ); //125 and 28
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 85); //125 and 35
  chassis.pid_wait();
}

void midPart(){
  chassis.pid_drive_set(22_in, DRIVE_SPEED); //drive towards long goal
  chassis.pid_wait();
  chassis.pid_turn_set(134_deg,TURN_SPEED); //224
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  topIntakeMotor.move(-10);
  chassis.pid_drive_set(40_in, DRIVE_SPEED+5); //drive towards cluster of balls 48 inches
  chassis.pid_wait_until(12_in);//at 35 inches switch to ballgoalspeed -15
  chassis.pid_speed_max_set(BALLGOAL_SPEED-30);
  chassis.pid_wait();

  //second 4 balls stack
  chassis.pid_turn_set(86_deg, TURN_SPEED+10); //turn toward other 4 balls stack 86 before
  chassis.pid_wait();
  middleIntakeMotor.move(127);
  chassis.pid_drive_set(53_in, DRIVE_SPEED+5); //drive towards long goal 53 old 
  chassis.pid_wait_until(21_in);//at 21 inches switch to ballgoalspeed -20
  chassis.pid_speed_max_set(BALLGOAL_SPEED-20);
  chassis.pid_wait();
  topIntakeMotor.move(-20);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED+10); //back away from balls a bit 8 inches 
  chassis.pid_wait();
  lilRaaahh.set(true); //prepare matchloader to score
  //middle goal scoring 
  chassis.pid_turn_set(45_deg,TURN_SPEED+10); //45
  //lilRaaahh.set(true);
  chassis.pid_wait();
  intake.move(-35);
  chassis.pid_drive_set(-21_in, DRIVE_SPEED+5); //drive back 23
  chassis.pid_wait_until(-20_in);
  middleGoalScore.set(true);
  chassis.pid_wait();
  //intake.move(0);
  // middleIntakeMotor.move(110);
  // //topIntakeMotor.move(-40);
  // //topIntakeMotor.move(-80); //old middle goal scoring 
  // pros::delay(2200); //delay for scoring at middle goal
  // middleIntakeMotor.move(90);
  // pros::delay(1500); //delay for scoring at middle goal

  // middleIntakeMotor.move(90); //90 bottom 
  // topIntakeMotor.move(-70); //-60 top 
  // pros::delay(3500); //3500

  middleIntakeMotor.move(90); //90 bottom 
  topIntakeMotor.move(-70); //-60 top 
  pros::delay(2900); //3500
  middleIntakeMotor.move(80); //90 bottom 
  topIntakeMotor.move(0); //-60 top 
  pros::delay(600); //3500
}

void park(){
  chassis.drive_angle_set(180_deg);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in,DRIVE_SPEED+10); //7
  chassis.pid_wait();
  chassis.pid_turn_set(220_deg, TURN_SPEED); //228
  chassis.pid_wait();
  chassis.pid_drive_set(35_in,DRIVE_SPEED+10); //33 +10 and 42
  //chassis.pid_wait();
  pros::delay(1100);
  middleIntakeMotor.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING, 265_deg, 120); //265
  //chassis.pid_wait();
  pros::delay(600);
  // chassis.pid_drive_set(35_in, 125); //125 and 35
  // chassis.pid_wait();
  chassis.pid_drive_set(30_in, 85); //125 and 28
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 85); //125 and 35
  chassis.pid_wait();
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . .  