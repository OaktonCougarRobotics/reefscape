// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_joystick = new Joystick(1);
  Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  Trigger toPoseButton = new Trigger(() -> m_joystick.getRawButton(1)); // Work in Progress - Horatio
  Trigger zeroWheels = new Trigger(() -> m_joystick.getRawButton(2));
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
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
    m_drivetrain.setDefaultCommand(m_drivetrain.driveCommand(() -> m_joystick.getRawAxis(1) * -1,
        () -> m_joystick.getRawAxis(0) * -1,
        () -> m_joystick.getRawAxis(2) * -1));

    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
    toPoseButton.onTrue(Commands.runOnce(() -> m_drivetrain.toPose(new Pose2d(11, 0, new Rotation2d(0, 0)))));
    zeroWheels.onTrue(Commands.runOnce(m_drivetrain::zeroWheels));
  }

  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_drivetrain.getAutonomousCommand("straight");
  }

  public void setMotorBrake(boolean brake) {
    m_drivetrain.setMotorBrake(brake);
  }

}