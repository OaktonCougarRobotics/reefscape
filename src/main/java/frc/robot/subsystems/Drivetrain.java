// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.text.DecimalFormat;

import org.dyn4j.geometry.Vector2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.AT;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.imu.*;

public class Drivetrain extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */

  DecimalFormat decF = new DecimalFormat("0.000");
  // limiting linear and angular velocity
  // private static final double MAX_LINEAR_VELOCITY = 3.0; //max speed in
  // meters/second
  // private static final double MAX_ANGULAR_VELOCITY = 2.0; //max speed in
  // radians/second
  double angleThreshold = 10;
  double positionThreshold = 0.2;
  // Limelight stuff, NOT YAGSL, VISION MADE THIS IT MAY BE BROKEN
  public final Translation2d m_frontLeftLocation = new Translation2d(Inches.of(12.125), Inches.of(12.125));
  public final Translation2d m_frontRightLocation = new Translation2d(Inches.of(12.125),
      Inches.of(0).minus(Inches.of(12.125)));
  public final Translation2d m_backLeftLocation = new Translation2d(Inches.of(0).minus(Inches.of(12.125)),
      Inches.of(12.125));
  public final Translation2d m_backRightLocation = new Translation2d(Inches.of(0).minus(Inches.of(12.125)),
      Inches.of(0).minus(Inches.of(12.125)));

  public final SwerveIMU m_gyro;
  // new AnalogGyro(0); //CHANGE THIS TO THE CORRECT PORT

  public SwerveModule m_frontLeft;
  public SwerveModule m_frontRight;
  public SwerveModule m_backLeft;
  public SwerveModule m_backRight;
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  // do all the consturction in init, remove the final, only declare local
  // variables here
  public final SwerveDrivePoseEstimator m_poseEstimator;

  public final double radiusOfRotation = 2;// Used for toAprilTag to find radius of rotation around the reef

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(File directory) {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(0),
              Meter.of(0)),
              Rotation2d.fromDegrees(0)));

      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    // SwerveDrive.invertOdometry();
    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via
                                            //
    swerveDrive.setCosineCompensator(false);// FIX: test out, maybe implement?

    swerveDrive.setAngularVelocityCompensation(true,
        true,
        -0.05); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.

    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
    // VISION, NOT YAGSL(NO CLUE IF THESE INDEXES ARE RIGHT)

    m_frontLeft = swerveDrive.getModules()[0];
    m_frontRight = swerveDrive.getModules()[1];
    m_backLeft = swerveDrive.getModules()[2];
    m_backRight = swerveDrive.getModules()[3];

    m_gyro = swerveDrive.getGyro();

    for (String key: swerveDrive.getModuleMap().keySet())
      System.out.println(key+": "+swerveDrive.getModuleMap().get(key).getAbsolutePosition());
      // swerveDrive.getModuleMap().get("frontleft").setAngle(90);



    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation3d().toRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotation) {

    return run(() -> {
      swerveDrive.driveFieldOriented(new ChassisSpeeds(
          deadzone(translationX.getAsDouble(), Constants.OperatorConstants.X_DEADBAND)
              * swerveDrive.getMaximumChassisVelocity(),
          deadzone(translationY.getAsDouble(), Constants.OperatorConstants.Y_DEADBAND)
              * swerveDrive.getMaximumChassisVelocity(),
          deadzone(angularRotation.getAsDouble(), Constants.OperatorConstants.Z_DEADBAND)
              * swerveDrive.getMaximumChassisAngularVelocity()),
          new Translation2d());
    });
    // Make the robot move
    // swerveDrive.drive(
    // new Translation2d(deadzone(translationX.getAsDouble(), 0.05) *
    // swerveDrive.getMaximumChassisVelocity(),
    // deadzone(translationY.getAsDouble(), 0.05) *
    // swerveDrive.getMaximumChassisVelocity()),

    // deadzone(angularRotation.getAsDouble(), 0.05) *
    // swerveDrive.getMaximumChassisAngularVelocity(),
    // true,
    // false);

    // swerveDrive.driveFieldOriented(
    // )
    // });
  }

  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation3d().toRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
      }

    boolean rejectVision = false;

    //LimelightHelpers.SetRobotOrientation("limelight",
        //m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        //0, 0, 0, 0, 0);
    //LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // if (Math.abs(m_gyro.getYawAngularVelocity().magnitude()) > 720) // if our
    // // angular velocity is greater than 720
    // // degrees per second, ignore vision updates
    // {
    //   rejectVision = true;
    // }
  //   if (mt2.tagCount == 0) {
  //     SmartDashboard.putBoolean("mt2Null?", true);
  //     rejectVision = true;
  //   } else {
  //     SmartDashboard.putBoolean("mt2Null?", false);
  //   }
  //   if (mt2.tagCount == 0) { // || mt2.pose == null) {
  //     rejectVision = true;
  //   }
  //   if (!rejectVision) {
  //     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
  //     m_poseEstimator.addVisionMeasurement(
  //         mt2.pose,
  //         mt2.timestampSeconds);
  //   } else if (rejectVision) {
  //     m_poseEstimator.update(
  //         m_gyro.getRotation3d().toRotation2d(),
  //         new SwerveModulePosition[] {
  //             m_frontLeft.getPosition(),
  //             m_frontRight.getPosition(),
  //             m_backLeft.getPosition(),
  //             m_backRight.getPosition()
  //         });
  //   }
  // }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public static double deadzone(double num, double deadband) {
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }

  public void toPose(Pose2d targetPose) {
    PIDController xController = new PIDController(10, 0, 0);
    PIDController yController = new PIDController(10, 0, 0);
    PIDController thetaController = new PIDController(4.5, 0, 3);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double xSpeed = xController.calculate(getX(), targetPose.getX());
    double ySpeed = yController.calculate(getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(getRotation().getRadians(), targetPose.getRotation().getRadians());
    while (!m_poseEstimator.getEstimatedPosition().equals(targetPose)
        && (Math.abs(xSpeed) > 0.1 || Math.abs(ySpeed) > 0.1 || Math.abs(thetaSpeed) > 0.05)) {
      swerveDrive.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));
      updateOdometry();
      xSpeed = xController.calculate(getX(), targetPose.getX());
      ySpeed = yController.calculate(getY(), targetPose.getY());
      thetaSpeed = thetaController.calculate(getRotation().getRadians(), targetPose.getRotation().getRadians());
    }
    xController.close();
    yController.close();
    thetaController.close();
  }

  // Create a list of waypoints from poses. Each pose represents one waypoint. The
  // rotation component of the pose should be the direction of travel. Do not use
  // holonomic rotation.
  public Command driveToPose(Pose2d pose) {
    // if (!within(pose, getPose())) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(m_poseEstimator.getEstimatedPosition(), pose);

    System.out.println("x: " + m_poseEstimator.getEstimatedPosition().getX() + " y: "
        + m_poseEstimator.getEstimatedPosition().getY() + " rotation: "
        + m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    // the constraints for this path
    PathConstraints constraints = new PathConstraints(3, 1, 4 * Math.PI, 3 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, pose.getRotation()));

    path.preventFlipping = true; // Prevent the path from being flipped if the coordinates are already correct

    AutoBuilder.followPath(path).addRequirements(this);
    // AutoBuilder.followPath(path).schedule();

    return AutoBuilder.followPath(path);
    // } else {
    // toPose(pose);
    // }
  }

  public boolean within(Pose2d pose, Pose2d pose2) {
    return Math.abs(pose.getX() - pose2.getX()) < positionThreshold &&
        Math.abs(pose.getY() - pose2.getY()) < positionThreshold;
  }

  public double findAngleRad(Pose2d reef, Pose2d endPosition) {
    Vector2 reefToBot = new Vector2(m_poseEstimator.getEstimatedPosition().getX() - reef.getX(),
        m_poseEstimator.getEstimatedPosition().getY() - reef.getY());
    Vector2 reefToEndPosition = new Vector2(endPosition.getX() - reef.getX(), endPosition.getY() - reef.getY());
    double dotProduct = reefToBot.dot(reefToEndPosition);
    double reefToBotMag = reefToBot.getMagnitude();
    double reefToEndPositionMag = reefToEndPosition.getMagnitude();
    return Math.acos(dotProduct / (reefToBotMag * reefToEndPositionMag));
  }

  // If auto doesnt work, finds the starting point of the robot's circular
  // rotation
  // around the reef
  public Pose2d findPoseA(Pose2d reef, AT aprilTag) {
    double dx = m_poseEstimator.getEstimatedPosition().getX() - reef.getX();
    double dy = m_poseEstimator.getEstimatedPosition().getY() - reef.getY();
    double theta = Math.atan(dy / dx);
    double ax = reef.getX() + radiusOfRotation * Math.cos(theta);
    double ay = reef.getY() + radiusOfRotation * Math.sin(theta);
    Pose2d a = new Pose2d(ax, ay, new Rotation2d(theta + Math.PI));

    return a;
  }

  public Pose2d findB(AT aprilTag) {
    return new Pose2d(
        (aprilTag.getPose().getX() + radiusOfRotation * Math.cos(aprilTag.getPose().getRotation().getRadians())),
        (aprilTag.getPose().getY() + radiusOfRotation * Math.sin(aprilTag.getPose().getRotation().getRadians())),
        aprilTag.getOffestPose().getRotation());
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public int closestAprilTag() {
    double min = Math
        .sqrt(Math.pow(m_poseEstimator.getEstimatedPosition().getX() - Constants.aprilPose[1].getPose().getX(), 2)
            + Math.pow(m_poseEstimator.getEstimatedPosition().getY() - Constants.aprilPose[1].getPose().getY(), 2));
    int index = 0;
    for (int i = 1; i <= 22; i++) {
      if (Math.sqrt(
          Math.pow(m_poseEstimator.getEstimatedPosition().getX() - Constants.aprilPose[i].getPose().getX(), 2) + Math
              .pow(m_poseEstimator.getEstimatedPosition().getY() - Constants.aprilPose[i].getPose().getY(), 2)) < min) {
        min = Math
            .sqrt(Math.pow(m_poseEstimator.getEstimatedPosition().getX() - Constants.aprilPose[i].getPose().getX(), 2)
                + Math.pow(m_poseEstimator.getEstimatedPosition().getY() - Constants.aprilPose[i].getPose().getY(), 2));
        index = i;
      }
    }
    System.out.println(index);
    return index;
  }

  public void toClosestAprilTag() {
    toPose(Constants.aprilPose[closestAprilTag()].getOffestPose());
  }

  public double getX() {
    return m_poseEstimator.getEstimatedPosition().getX();
  }

  public double getY() {
    return m_poseEstimator.getEstimatedPosition().getY();
  }

  public Rotation2d getRotation() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateTelemetry();
    SmartDashboard.putNumber("Front left volt", swerveDrive.getModuleMap().get("frontleft").getAngleMotor().getVoltage());
    for (String key: swerveDrive.getModuleMap().keySet()) {
      SmartDashboard.putNumber(key + "rawabsolute",swerveDrive.getModuleMap().get(key).getRawAbsolutePosition());
      // System.out.println(key+": "+swerveDrive.getModuleMap().get(key).getAbsolutePosition());
      // System.out.println(key+": "+swerveDrive.getModuleMap().get(key).getRelativePosition());
    }
  }

  public void updateTelemetry() {
    for (String key : swerveDrive.getModuleMap().keySet()) {
      SmartDashboard.putNumber(key, swerveDrive.getModuleMap().get(key).getAbsolutePosition());
    }
    SmartDashboard.putString("position",
        "(" + decF.format(swerveDrive.getPose().getX()) + ", " + decF.format(swerveDrive.getPose().getY()) + ")");
    SmartDashboard.putNumber("theta", swerveDrive.getOdometryHeading().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void zeroWheels() {
    for (String key : swerveDrive.getModuleMap().keySet())
      swerveDrive.getModuleMap().get(key).setAngle(0);
  }

  public void changeYawAutoRed() {
    swerveDrive.setGyro(swerveDrive.getGyro().getRotation3d().minus(new Rotation3d(new Rotation2d(180 - 126.591))));
  }
}