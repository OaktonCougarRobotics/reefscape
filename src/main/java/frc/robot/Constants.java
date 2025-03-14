// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import frc.AT;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static AT april1 = new AT(657.37, 25.80, 126, 1);// static Pose2d april1 = new Pose2d(657.37, 25.80, new
                                                          // Rotation2d(0.0));
  public static AT april2 = new AT(657.37, 291.20, 234, 1);// static Pose2d april2 = new Pose2d(657.37, 291.20, new
                                                           // Rotation2d(0.0));
  public static AT april3 = new AT(455.15, 317.15, 270, 1);// static Pose2d april3 = new Pose2d(455.15, 317.15, new
                                                           // Rotation2d(0.0));
  public static AT april4 = new AT(365.20, 241.64, 0, 1);
  public static AT april5 = new AT(365.20, 75.39, 0, 1);
  public static AT april6 = new AT(530.49, 130.17, 300, 1);
  public static AT april7 = new AT(546.87, 158.5, 0, 1);
  public static AT april8 = new AT(530.49, 186.83, 60, 1);
  public static AT april9 = new AT(497.77, 186.83, 120, 1);
  public static AT april10 = new AT(481.39, 158.5, 180, 1);
  public static AT april11 = new AT(497.77, 130.17, 240, 1);
  public static AT april12 = new AT(33.51, 25.80, 54, 1);
  public static AT april13 = new AT(33.51, 291.20, 306, 1);
  public static AT april14 = new AT(325.68, 241.64, 180, 1);
  public static AT april15 = new AT(325.68, 75.39, 180, 1);
  public static AT april16 = new AT(235.73, -0.15, 90, 1);
  public static AT april17 = new AT(160.39, 130.17, 240, 1);
  public static AT april18 = new AT(144.00, 158.50, 180, 1);
  public static AT april19 = new AT(160.39, 186.83, 120, 1);
  public static AT april20 = new AT(193.10, 186.83, 60, 1);
  public static AT april21 = new AT(209.49, 158.50, 0, 1);
  public static AT april22 = new AT(193.10, 130.17, 300, 1);

  public static AT[] aprilPose = new AT[] { null, april1, april2, april3, april4, april5, april6, april7, april8,
      april9, april10, april11, april12, april13, april14, april15, april16, april17, april18, april19, april20,
      april21, april22 };

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static final double STEER_ENCODER_POSITION_CONSTANT = 2.0 * Math.PI; // STEER_REDUCTION
  public static final double STEER_ENCODER_VELOCITY_CONSTANT = STEER_ENCODER_POSITION_CONSTANT * 10.0;

  public static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  public static final int ENCODER_RESET_ITERATIONS = 500;

  public static final double WHEEL_DIAMETER = 0.10001;

  // ROBOT_WHEELBASE Configuration
  public static final double DRIVETRAIN_TRACKWIDTH_METERS_ROBOT_WHEELBASE = 0.5644;
  public static final double DRIVETRAIN_WHEELBASE_METERS_ROBOT_WHEELBASE = 0.666;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ROBOT_WHEELBASE = 32;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ROBOT_WHEELBASE = 35;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROBOT_WHEELBASE = -137;
  public static final int FRONT_LEFT_MODULE_ENCODER_PORT_ROBOT_WHEELBASE = 3;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ROBOT_WHEELBASE = 42;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ROBOT_WHEELBASE = 34;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROBOT_WHEELBASE = -84.67;
  public static final int FRONT_RIGHT_MODULE_ENCODER_PORT_ROBOT_WHEELBASE = 1;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ROBOT_WHEELBASE = 41;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR_ROBOT_WHEELBASE = 36;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET_ROBOT_WHEELBASE = -140.6;
  public static final int BACK_LEFT_MODULE_ENCODER_PORT_ROBOT_WHEELBASE = 2;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ROBOT_WHEELBASE = 37;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ROBOT_WHEELBASE = 33;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROBOT_WHEELBASE = -194.98;
  public static final int BACK_RIGHT_MODULE_ENCODER_PORT_ROBOT_WHEELBASE = 0;

  public static final double LIMELIGHT_HEIGHT_ROBOT_WHEELBASE = 38.57625; // cm
  public static final double TARGET_HEIGHT_TALL_ROBOT_WHEELBASE = 69; // cm
  public static final double TARGET_HEIGHT_SHORT_ROBOT_WHEELBASE = 31.59125; // cm
  public static final double LIMELIGHT_ANGLE_ROBOT_WHEELBASE = 2.25; // degrees

  // ANATOLI Configuration
  public static final double DRIVETRAIN_WHEELBASE_METERS_ANATOLI = 0.61595;
  public static final double DRIVETRAIN_TRACKWIDTH_METERS_ANATOLI = 0.61806;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ANATOLI = 41;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ANATOLI = 36;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET_ANATOLI = 4.090;
  public static final int FRONT_LEFT_MODULE_ENCODER_PORT_ANATOLI = 2;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ANATOLI = 42;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ANATOLI = 34;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ANATOLI = 3.604;
  public static final int FRONT_RIGHT_MODULE_ENCODER_PORT_ANATOLI = 1;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ANATOLI = 37;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR_ANATOLI = 52;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET_ANATOLI = 3.459;
  public static final int BACK_LEFT_MODULE_ENCODER_PORT_ANATOLI = 3;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ANATOLI = 53;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ANATOLI = 59;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET_ANATOLI = 2.582;
  public static final int BACK_RIGHT_MODULE_ENCODER_PORT_ANATOLI = 0;

  public static final double MAX_VOLTAGE_WHEN_OFFSET_ANATOLI = 4.927;
  public static final int CLIMB_MOTOR_ANATOLI = 31;

  public static final double TARGET_HEIGHT_TALL = 69; // cm
  public static final double TARGET_HEIGHT_SHORT = 31.59125; // cm

  public static final double DRIVE_ENCODER_POSITION_CONSTANT = Math.PI * WHEEL_DIAMETER; // DRIVE_REDUCTION
  public static final double DRIVE_ENCODER_VELOCITY_CONSTANT = DRIVE_ENCODER_POSITION_CONSTANT * 10;

  public static final double ENCODER_VOLTAGE_TO_DEGREE = 360 / 4.809;

  public static final String COB_KEY_IS_RED = "/FMSInfo/IsRedAlliance";
  public static final String COB_KEY_DISTANCE = "/COB/distance";
  public static final String COB_KEY_BOT_POSE_FRONT = "/limelight-front/botpose";
  public static final String COB_KEY_BOT_POSE_BACK = "/limelight-back/botpose";
  public static final String COB_KEY_BOT_POSE_BLUE_FRONT = "/limelight-front/botpose_wpiblue";
  public static final String COB_KEY_BOT_POSE_BLUE_BACK = "/limelight-back/botpose_wpiblue";
  public static final String COB_KEY_TV_FRONT = "/limelight-front/tv";
  public static final String COB_KEY_TV_BACK = "/limelight-back/tv";
  public static final String COB_KEY_TX_FRONT = "/limelight-front/tx";
  public static final String COB_KEY_TX_BACK = "/limelight-back/tx";
  public static final String COB_KEY_TA_FRONT = "/limelight-front/ta";
  public static final String COB_KEY_TA_BACK = "/limelight-back/ta";
  public static final String COB_KEY_MATCHTIME = "/COB/matchTime";

  // ARM_SUBSYSTEM Configuration
  // All -1 as filler values, define all the motor id's and DIO id later
  public static final int CORALFLYWHEEL_MOTOR = -1; // FIX
  public static final int CORALPIVOT_MOTOR= -1; // FIX
  public static final int ALGAEDRIVEMOTOR = -1;
  public static final int ALGAEPIVOTMOTOR = -1;
  public static final int TELESCOPE_MOTOR = -1;
  public static final int ELEVATOR_MOTOR = -1;
  // Figure out how to get ticks from the motor
  public static double BOTTOM_TURNS = -1;
  public static final double LOW_TARGET = -1;
  public static final double MID_TARGET = -1;
  public static final double HIGH_TARGET = -1;
  public static final double PICKUP_TARGET = -1;

  public static final double PIVOT_CAN_DIFFERENCE_BETWEEN_STARTING_AND_LEVEL = -1;
  public static final double PIVOT_GEAR_RATIO = 160.0;
  public static final double PIVOT_TICKS_PER_DEGREE = PIVOT_GEAR_RATIO * 2048.0 / 360.0;

  public static final double WRIST_GEAR_RATIO = 156.522;
  public static final double WRIST_TICKS_PER_DEGREE = WRIST_GEAR_RATIO * 2048.0 / 360.0;

  public static final double ARM_TOTAL_TICKS = 185684;
  public static final double ARM_TOTAL_DEGREES = 209.1;
  // public static final double CANCODER_MIN = 28.5;
  // public static final double CANCODER_MAX = 237.6;
  // public static final double CANCODER_ZERO = 126.3;

  public static final double PIVOT_HIGH = -1.28;
  public static final double PIVOT_TOTAL_ROTATIONS = 3.06;
  public static final double PIVOT_LOW = 1.78;

  public static final double STRINGPOT_LOW = 348;
  public static final double STRINGPOT_TOP = 704;
  public static final double STRINGPOT_TOTAL_RANGE = 356;

  public static final double PIVOT_ROTATIONS_PER_STRINGPOT_UNITS = PIVOT_TOTAL_ROTATIONS / STRINGPOT_TOTAL_RANGE;
  public static final double ARM_LENGTH = 30.75;
  public static final double DIFF_BASE_PIVOT_STRINGPOT = 10;
  public static final double CLOSEUPSHOOTSTRINGPOT = 555;
  public static final double PICKUPSTRINGPOT = 420;
  public static final double PROTECTEDBLOCKSHOOT = 356;

  public static final double PIVOT_DFLT_VEL = 8000;
  public static final double PIVOT_DFLT_ACC = 10000;
  public static final double WRIST_DFLT_VEL = 14000;
  public static final double WRIST_DFLT_ACC = 28000;
  public static final double PIVOT_ACC_DIVISOR = 3.5;

  public static final double ANATOLI_CHASSIS_WIDTH = 0.3; // meters

  // Button IDs
  public static final int DUSTPANUP_LIMIT = 0;
  public static final int SHOOTER_SPEED = 1;
  public static final int FLYWHEEL_SWITCH = 1;
  public static final int INTAKE_SWITCH = 2;
  public static final int AIM_BUTTON = 3;
  public static final int TEST_BIG_YELLOW_BUTTON = 4;
  public static final int SERVO_SHOOT = 5;
  public static final int SHOOTER_LOCK_POWER = 6;
  public static final int CLOSE_SHOOT_BUTTON = 7;
  public static final int PICKUP_BUTTON = 8;
  public static final int PROTECTED_BLOCK_SHOOT = 12;
  public static final int NUKE_SWITCH_4 = 13;
  public static final int NUKE_SWITCH_3 = 14;
  public static final int NUKE_SWITCH_2 = 15;
  public static final int NUKE_SWITCH_1 = 16;
  public static final int SHOOTER_DOWN = 17;
  public static final int SHOOTER_UP = 18;
  public static final int CLIMB_UP = 19;
  public static final int CLIMB_DOWN = 20;
  public static final int DUSTPAN_UP = 21;
  public static final int DUSTPAN_DOWN = 22;
  


  public static final int ARM_UP = -1;
  public static final int ARM_DOWN = -1;
  public static final int UPPER_LIMIT = -1;
  public static final int LOWER_LIMIT = 0;

  public static final double LIMELIGHT_HEIGHT = 0.238125; // m from the bottom
  public static final double LIMELIGHT_YTHETA = 60; // off the horizontal
  public static final double LIMELIGHT_DISPLACEMENT = 0.1524; // m from the front
  public static final double LIMELIGHT_CENTER_DISPLACEMENT = 0.1397; // m from the center of the front

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double X_DEADBAND = 0.0; //0.075
    public static final double Y_DEADBAND = 0.0;
    public static final double Z_DEADBAND = 0.0; //0.03
    public static final double TURN_CONSTANT = 6;
  }
}