// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AngleCorrection;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SpinFeeder;
import frc.robot.subsystems.Drivetrain;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Joystick object
  public final Joystick m_joystick = new Joystick(1);
  // Triggers on the joystick
  private Trigger m_inputSpin = new Trigger(() -> m_joystick.getRawButton(6));
  private Trigger m_navxReset = new Trigger(() -> m_joystick.getRawButton(3));
  private Trigger m_toPose = new Trigger(() -> m_joystick.getRawButton(2)); // Work in Progress - Horatio
  private Trigger m_setTargetPose = new Trigger(() -> m_joystick.getRawButton(1)); // Work in Progress - Horatio

  // feeder motor
  private TalonSRX m_feederMotor = new TalonSRX(22);

  // elevator, wrist, and flywheel
  public TalonFX m_elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR);
  // public TalonFX m_wristMotor = new NEO SparkMAX(); ???
  public TalonSRX m_flywheelMotor = new TalonSRX(Constants.FLYWHEEL_MOTOR);

  SpinFeeder spinFeederCommand = new SpinFeeder(m_feederMotor);
  Pose2d test = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180)));
  DriveCommand driveCommand = new DriveCommand(
      m_drivetrain,
      () -> m_joystick.getRawAxis(1) * -1,
      () -> m_joystick.getRawAxis(0) * -1,
      () -> m_joystick.getRawAxis(2) * -1);
  AngleCorrection angleCorrection = new AngleCorrection(m_drivetrain, () -> {
    return test;
  });

  public RobotContainer() {
    NamedCommands.registerCommand("TestMe", Commands.runOnce(() -> {
      SmartDashboard.putNumber("testingNamedCommands", 6232025);
    }));
    NamedCommands.registerCommand("spin", spinFeederCommand);

    m_drivetrain.setupPathPlanner();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mapping s. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(driveCommand);

    m_setTargetPose.onTrue(Commands.runOnce(() -> {
      test = m_drivetrain.getPose();
    }));

    m_navxReset.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));

    /*
    m_toPose.whileTrue(
      Commands.runOnce(
        ()->{
          if(m_drivetrain.within(m_drivetrain.getPose(), test))
            angleCorrection.schedule();
          else
            m_drivetrain.driveToPose(test);
        }
      )
      // angleCorrection
    // () -> {
    //   if (!m_drivetrain.within(test, m_drivetrain.getPose())) {
    //     m_drivetrain.driveToPose(test);
    //   } else {
    //     angleCorrection.schedule();
    //   }
    // }
    // )
    );

    */

    m_toPose.onTrue(
      Commands.runOnce(
        ()->{
          Command drive = m_drivetrain.driveToPose(test);
          drive.schedule();
          if (m_drivetrain.within(m_drivetrain.getPose(), test)) {
            angleCorrection.schedule();
          } else if (m_joystick.getRawAxis(1) > OperatorConstants.X_DEADBAND || m_joystick.getRawAxis(0) > OperatorConstants.X_DEADBAND || m_joystick.getRawAxis(2) > OperatorConstants.X_DEADBAND) {
            // CommandScheduler.getInstance().cancel(angleCorrection);
            // CommandScheduler.getInstance().cancel(drive);
            CommandScheduler.getInstance().cancelAll();
            // angleCorrection.cancel();
            // drive.cancel();
          } //else {
          //   drive.schedule();
          // }
        }
      )
    );
    // m_toPose.whileFalse(Commands.run(()->{
    //   angleCorrection.end(true);
      
    // })

    // );

    // () -> m_drivetrain.driveToPose(test)).andThen(angleCorrection)
    // .andThen(Commands.runOnce(() -> {
    // PIDController thetaController = new PIDController(.5, 0, .00001);
    // // thetaController.enableContinuousInput(-Math.PI, Math.PI); //NOT SURE WHAT
    // // THIS DOES, FIX OR REMOVE
    // double thetaSpeed =
    // thetaController.calculate(m_drivetrain.getRotation().getRadians(),
    // test.getRotation().getRadians());
    // while (Math.abs(thetaSpeed) > 0.1
    // && m_drivetrain.getPose().getRotation().getDegrees() -
    // test.getRotation().getDegrees() > 7) {
    // m_drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0,
    // thetaSpeed));
    // thetaSpeed =
    // thetaController.calculate(m_drivetrain.getRotation().getRadians(),
    // test.getRotation().getRadians());
    // }
    // }))

    m_inputSpin.whileTrue(spinFeederCommand);
  }

  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  /**
   * 
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_drivetrain.getAutonomousCommand("Blue rightest StartPos");
  }

  public static double deadzone(double num, double deadband) {
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }
}