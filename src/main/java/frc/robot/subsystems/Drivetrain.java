// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

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
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
              // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the
                                         // offsets onto it. Throws warning if not possible
    System.out.println("front left: "+swerveDrive.getModuleMap().get("frontleft").getRawAbsolutePosition());
    System.out.println("fron right: "+swerveDrive.getModuleMap().get("frontright").getRawAbsolutePosition());
    System.out.println("back left: "+swerveDrive.getModuleMap().get("backleft").getRawAbsolutePosition());
    System.out.println("backright: "+swerveDrive.getModuleMap().get("backright").getRawAbsolutePosition());
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
        deadzone(translationX.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity(),
        deadzone(translationY.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity(),
        deadzone(angularRotation.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisAngularVelocity()));
    });
      // Make the robot move
      // swerveDrive.drive(
      //   new Translation2d(deadzone(translationX.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity(),
      //                                     deadzone(translationY.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisVelocity()),
                                          
      //                   deadzone(angularRotation.getAsDouble(), 0.05) * swerveDrive.getMaximumChassisAngularVelocity(),
      //     true,
      //        false);

    //   swerveDrive.driveFieldOriented(
    //     )
    // });
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

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}