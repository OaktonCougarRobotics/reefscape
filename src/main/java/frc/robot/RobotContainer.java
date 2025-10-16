// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ArmSetpointCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;


public class RobotContainer {
  // Drivetrain
  private final Drivetrain m_drivetrain = new Drivetrain(
    new File(Filesystem.getDeployDirectory(),"swerve"));
  // Arm 
  private final Arm m_arm = new Arm(
    30,
    33,
    15//18 in 17 out
  );  
  // Button Board
  public final Joystick m_buttonBoard = new Joystick(0);
  // Triggers for Buttons on Button Board
  private final Trigger defaultTrigger = new Trigger(() -> m_buttonBoard.getRawButton(8));
  private final Trigger intakeTrigger = new Trigger(() -> m_buttonBoard.getRawButton(7));
  private final Trigger l2Trigger = new Trigger(() -> m_buttonBoard.getRawButton(6));
  private final Trigger l3Trigger = new Trigger(() -> m_buttonBoard.getRawButton(5));
  private final Trigger l4Trigger = new Trigger(() -> m_buttonBoard.getRawButton(4));
  
  private final Trigger intakeInTrigger = new Trigger(() -> m_buttonBoard.getRawButton(18));
  private final Trigger intakeOutTrigger = new Trigger(() -> m_buttonBoard.getRawButton(17));
  
  // Joystick
  public final Joystick m_joystick = new Joystick(1);
  // Triggers for Buttons on joystick
  private final Trigger inputSpin = new Trigger(() -> m_joystick.getRawButton(6));
  private final Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private final Trigger zeroWheels = new Trigger(() -> m_joystick.getRawButton(2));



  // desired arm states: use for commands
  private ElevatorState desiredElevatorState = ElevatorState.DEFAULT;
  private WristState desiredWristState = WristState.DEFAULT;
  private IntakeState desiredIntakeState = IntakeState.DEFAULT;

  // private final Trigger wristOut =new Trigger(() -> m_joystick.getRawButton(4));
  // private final Trigger wristIn =new Trigger(() -> m_joystick.getRawButton(6));
  // Default Drive Command
  private final DriveCommand m_driveCommand = new DriveCommand(
    m_drivetrain,
    () -> m_joystick.getRawAxis(1) * -1,
    () -> m_joystick.getRawAxis(0) * -1,
    () -> m_joystick.getRawAxis(2) * -1
  );
  private final ArmSetpointCommand wristConstCommand = new ArmSetpointCommand(
    m_arm,
    () -> desiredWristState.getPosition(),
    () -> desiredElevatorState.getPosition(),
    () -> desiredIntakeState.getVelocity()
  );
  public RobotContainer() {
    m_drivetrain.setupPathPlanner();
    configureBindings();
  }
  private void configureBindings() {
    m_drivetrain.setDefaultCommand( 
      m_driveCommand 
    );
    m_arm.setDefaultCommand(
      wristConstCommand
    );
    navxResetButton.onTrue(
      Commands.runOnce(m_drivetrain::zeroGyro)
    );
    zeroWheels.onTrue(
      Commands.runOnce(m_drivetrain::zeroWheels)
    );

    defaultTrigger.onTrue(Commands.runOnce(() -> {
      desiredElevatorState = ElevatorState.DEFAULT;
      desiredWristState = WristState.DEFAULT;
    }));
    intakeTrigger.onTrue(Commands.runOnce(() -> {
      desiredElevatorState = ElevatorState.INTAKE;
      desiredWristState = WristState.INTAKE;
    }));
    l2Trigger.onTrue(Commands.runOnce(() -> {
      desiredElevatorState = ElevatorState.L2;
      desiredWristState = WristState.L2;
    }));
    l3Trigger.onTrue(Commands.runOnce(() -> {
      desiredElevatorState = ElevatorState.L3;
      desiredWristState = WristState.L3;
    }));
    l4Trigger.onTrue(Commands.runOnce(() -> {
      desiredElevatorState = ElevatorState.L4;
      desiredWristState = WristState.L4;
    }));
    intakeInTrigger.onTrue(Commands.runOnce(() -> {
      desiredIntakeState = IntakeState.INTAKE;
    }));
    intakeInTrigger.onFalse(Commands.runOnce(() -> {
      desiredIntakeState = IntakeState.DEFAULT;
    }));
    intakeOutTrigger.onTrue(Commands.runOnce(() -> {
      desiredIntakeState = IntakeState.OUTTAKE;
    }));
    intakeOutTrigger.onFalse(Commands.runOnce(() -> {
      desiredIntakeState = IntakeState.DEFAULT;
    }));
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
    SmartDashboard.putNumber("wrist position", m_arm.getWrist().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator position", m_arm.getElevator().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("intake speed %",m_arm.getIntake().getMotorOutputPercent());
  }
}
/*
 * This enum represents the main setpoints that the arm elevator will be in 
 */
enum ElevatorState{
  DEFAULT(0),
  INTAKE(0),
  L1(0),
  L2(-2),
  L3(-29.36),
  L4(-82);
  private double position;
  ElevatorState(double position){
      this.position = position;
  }
  public double getPosition(){
      return position;
  }
}
/*
 * This enum represents the main setpoints that the arm wrist will be in 
 */
enum WristState{
  DEFAULT(0.163),
  INTAKE(-.182),
  L1(0.163),
  L2(-0.333),
  L3(-0.333),
  L4(-.400);
  private double position;
  WristState(double position){
      this.position = position;
  }
  public double getPosition(){
      return position;
  }
}
/*
 * This enum represents the main setpoints that the arm intake will be in 
 */
enum IntakeState{
  DEFAULT(0.0),
  INTAKE(-0.4),
  OUTTAKE(0.4);
  private double velocity;
  IntakeState(double velocity){
      this.velocity =velocity;
  }
  public double getVelocity(){
      return velocity;
  }
}