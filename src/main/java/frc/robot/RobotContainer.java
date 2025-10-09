// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.SpinFeeder;
import frc.robot.commands.WristConstCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  // Button board
  public final Joystick m_buttonBoard = new Joystick(0);
  // Joystick
  public final Joystick m_joystick = new Joystick(1);
  // Triggers on the joystick
  private Trigger inputSpin = new Trigger(() -> m_joystick.getRawButton(6));
  private Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private Trigger zeroWheels = new Trigger(() -> m_joystick.getRawButton(2));


  private Trigger wristOut =new Trigger(() -> m_joystick.getRawButton(4));
  private Trigger wristIn =new Trigger(() -> m_joystick.getRawButton(6));
  // feeder motor
  // private TalonSRX feederMotor = new TalonSRX(22);
  // Command spinFeederCommand = new SpinFeeder(feederMotor);
  private Arm m_arm = new Arm(30);
  DriveCommand m_driveCommand = new DriveCommand(
    m_drivetrain,
    () -> m_joystick.getRawAxis(1) * -1,
    () -> m_joystick.getRawAxis(0) * -1,
    () -> m_joystick.getRawAxis(2) * -1
  );
  private WristConstCommand wristConstCommand = new WristConstCommand(
    m_arm,
    -.367
  );
  private TalonFX feeder = new TalonFX(0);  
  public RobotContainer() {
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
    m_drivetrain.setDefaultCommand( 
      m_driveCommand 
    );
    m_arm.setDefaultCommand(
      wristConstCommand);

    navxResetButton.onTrue(
      Commands.runOnce(m_drivetrain::zeroGyro)
    );
    zeroWheels.onTrue(
      Commands.runOnce(m_drivetrain::zeroWheels)
    );


    wristOut.whileTrue(Commands.run(() -> m_arm.getWrist().setControl(new DutyCycleOut(-.1)), m_arm));
    // wristOut.onFalse(Commands.run(() -> m_arm.getWrist().setControl(new DutyCycleOut(0)), m_arm));
    // wristIn.whileTrue(Commands.run(() -> m_arm.getWrist().setControl(new DutyCycleOut(.1)), m_arm));
    // wristIn.onFalse(Commands.run(() -> m_arm.getWrist().setControl(new DutyCycleOut(0)), m_arm));
    
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
    return m_drivetrain.getAutonomousCommand("Scizo");
  }
    /**
   * Use this to run code that must be consistently called (e.g telemetry)
   */
  public void periodic(){
    SmartDashboard.putNumber("wrist duty cycle out", m_arm.getWrist().getPosition().getValueAsDouble());
  }
}