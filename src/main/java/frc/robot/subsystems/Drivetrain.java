// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.imu.*;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import java.lang.*;

public class Drivetrain extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meter.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  // limiting linear and angular velocity
  // private static final double MAX_LINEAR_VELOCITY = 3.0; //max speed in
  // meters/second
  // private static final double MAX_ANGULAR_VELOCITY = 2.0; //max speed in
  // radians/second

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

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(File directory) {

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to
    // meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");

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
    swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via
                                            // angle.
    // swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation);
    // // Disables cosine compensation for simulations since it causes discrepancies
    // not seen in real life.

    // swerveDrive.

    swerveDrive.setAngularVelocityCompensation(true,
        true,
        -0.05); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the
                                         // offsets onto it. Throws warning if not possible
    // VISION, NOT YAGSL(NO CLUE IF THESE INDEXES ARE RIGHT)
    m_frontLeft = swerveDrive.getModules()[0];
    m_frontRight = swerveDrive.getModules()[1];
    m_backLeft = swerveDrive.getModules()[2];
    m_backRight = swerveDrive.getModules()[3];

    m_gyro = swerveDrive.getGyro();

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

    System.out.println("Front left: " + swerveDrive.getModuleMap().get("frontleft").getRawAbsolutePosition());
    System.out.println("Front right: " + swerveDrive.getModuleMap().get("frontright").getRawAbsolutePosition());
    System.out.println("Back left: " + swerveDrive.getModuleMap().get("backleft").getRawAbsolutePosition());
    System.out.println("Back right: " + swerveDrive.getModuleMap().get("backright").getRawAbsolutePosition());
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
      // swerveDrive.drive(new ChassisSpeeds(translationX.getAsDouble() *
      // Constants.MAX_SPEED, translationY.getAsDouble() * Constants.MAX_SPEED,
      // Math.PI+(Math.PI*angularRotation.getAsDouble())));;
      swerveDrive.driveFieldOriented(new ChassisSpeeds(
          deadzone(translationX.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity(),
          deadzone(translationY.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity(),
          deadzone(angularRotation.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisAngularVelocity()));
    });
    // Make the robot move
    // swerveDrive.drive(new Translation2d(
    // deadzone(translationX.getAsDouble(), OperatorConstants.X_DEADBAND) *
    // swerveDrive.getMaximumChassisVelocity(),
    // deadzone(translationY.getAsDouble(), OperatorConstants.Y_DEADBAND) *
    // swerveDrive.getMaximumChassisVelocity()),
    // deadzone(angularRotation.getAsDouble(), OperatorConstants.Z_DEADBAND)
    // * swerveDrive.getMaximumChassisAngularVelocity(),
    // true,
    // false);
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

    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (Math.abs(m_gyro.getYawAngularVelocity().magnitude()) > 720) // if our angular velocity is greater than 720
                                                                    // degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2 == null) {
      SmartDashboard.putBoolean("mt2Null?", true);
      doRejectUpdate = true;
    } else {
      SmartDashboard.putBoolean("mt2Null?", false);
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    } else if (doRejectUpdate) {
      m_poseEstimator.update(
          m_gyro.getRotation3d().toRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });
    }
  }

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
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
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

  public void printOdometry() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    System.out.println("x=" + pose.getX() + ", y=" + pose.getY() + ", theta=" + pose.getRotation().getDegrees());
  }

  // private void setDrivetrainVelocity(double linearVelocity, double
  // angularVelocity) {
  // linearVelocity = limitVelocity(linearVelocity);
  // angularVelocity = limitAngularVelocity(angularVelocity);
  // }

  // private double limitVelocity(double velocity) {
  // return Math.min(Math.max(velocity, -MAX_LINEAR_VELOCITY),
  // MAX_LINEAR_VELOCITY);
  // }

  // private double limitAngularVelocity(double angularVelocity) {
  // return Math.min(Math.max(angularVelocity, -MAX_ANGULAR_VELOCITY),
  // MAX_ANGULAR_VELOCITY);
  // }

  // private Trajectory generateTrajectory(Pose2d startPose, Pose2d endPose,
  // TrajectoryConfig config) {
  // return TrajectoryGenerator.generateTrajectory(
  // startPose,
  // List.of(),
  // endPose,
  // config
  // );
  // }

  // Follow the generated trajectory
  // private void followTrajectory(Trajectory trajectory) {
  // // Simple path-following using a PIDController for both x and theta (angular)
  // try (PIDController xController = new PIDController(1.0, 0, 0);
  // PIDController thetaController = new PIDController(1.0, 0, 0)) {

  // for (Trajectory.State state : trajectory.getStates()) {
  // // Update the robot’s velocity to follow the trajectory
  // double xError = state.poseMeters.getX() -
  // m_poseEstimator.getEstimatedPosition().getX();
  // double yError = state.poseMeters.getY() -
  // m_poseEstimator.getEstimatedPosition().getY();
  // double thetaError = state.poseMeters.getRotation().getDegrees() -
  // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();

  // double linearVelocity = xController.calculate(xError) + yError; // Using both
  // x and y errors
  // double angularVelocity = thetaController.calculate(thetaError);

  // setDrivetrainVelocity(linearVelocity, angularVelocity);
  // }
  // }
  // }

  public void toPose(Pose2d targetPose) {
    boolean autoWorks = false;
    if (!autoWorks)// Work in progress - Horatio
    {
      PIDController xController = new PIDController(1, 0, 0);
      PIDController yController = new PIDController(1, 0, 0);
      PIDController thetaController = new PIDController(1, 0, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      double xSpeed = xController.calculate(m_poseEstimator.getEstimatedPosition().getX(), targetPose.getX());
      double ySpeed = xController.calculate(m_poseEstimator.getEstimatedPosition().getY(), targetPose.getY());
      double thetaSpeed = xController.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
          targetPose.getRotation().getRadians());

      swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));

    }

    if (autoWorks) {
      Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
      // Create a list of waypoints from poses. Each pose represents one waypoint.
      // The rotation component of the pose should be the direction of travel. Do not
      // use holonomic rotation.
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()), // waypoints will always have
                                                                                         // at
                                                                                         // minimum two pose2ds (current
                                                                                         // and target)
          new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation()));

      PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this
                                                                                             // path.
      // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
      // You can also use unlimited constraints, only limited by motor torque and
      // nominal battery voltage

      // Create the path using the waypoints created above
      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                // be null for on-the-fly paths.
          new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If
                                                          // using a differential drivetrain, the rotation will have no
                                                          // effect.
      );

      // Prevent the path from being flipped if the coordinates are already correct
      path.preventFlipping = true;
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void periodic() {
    updateOdometry();
    printOdometry();
    // This method will be called once per scheduler run
    System.out.println("theta:" + swerveDrive.getOdometryHeading().getDegrees());
    System.out.println("x:" + swerveDrive.getPose().getX());
    System.out.println("y:" + swerveDrive.getPose().getY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}