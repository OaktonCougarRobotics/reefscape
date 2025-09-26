// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.SpinFeeder;
import frc.robot.subsystems.Drivetrain;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // need actual hardware location of button board
  int x = 1;
  public final Joystick m_buttonBoard = new Joystick(x);
  // button board buttons
  // four extra buttons not declared
  public final Trigger m_climbLeft = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_climbRight = new Trigger(() -> m_buttonBoard.getRawButton(x));
  // manual control button
  public final Trigger m_manualWrist = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_manualElevatorLift = new Trigger(() -> m_buttonBoard.getRawButton(x));
  // algae button
  public final Trigger m_algae = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_intakeSwitch = new Trigger(() -> m_buttonBoard.getRawButton(x));
  // different levels
  public final Trigger m_startingArmPos = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_coralStationPos = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_l4 = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_l3 = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_l2 = new Trigger(() -> m_buttonBoard.getRawButton(x));
  public final Trigger m_l1 = new Trigger(() -> m_buttonBoard.getRawButton(x));
  // vision button
  public final Trigger m_horatioMagic = new Trigger(() -> m_buttonBoard.getRawButton(x));

  // Joystick object
  public final Joystick m_joystick = new Joystick(1);
  // Triggers on the joystick
  private Trigger inputSpin = new Trigger(() -> m_joystick.getRawButton(6));
  private Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private Trigger toPoseButton = new Trigger(() -> m_joystick.getRawButton(1)); // Work in Progress - Horatio
  private Trigger zeroWheels = new Trigger(() -> m_joystick.getRawButton(2));
  // feeder motor
  private TalonSRX feederMotor = new TalonSRX(22);
  Command spinFeederCommand = new SpinFeeder(feederMotor);

  DriveCommand driveCommand = new DriveCommand(
      m_drivetrain,
      () -> m_joystick.getRawAxis(1) * -1,
      () -> m_joystick.getRawAxis(0) * -1,
      () -> m_joystick.getRawAxis(2) * -1);

  public RobotContainer() {
    // NamedCommands.registerCommand("TestMe", Commands.runOnce(() ->
    // SmartDashboard.putNumber("AAAA", 911)));
    NamedCommands.registerCommand("Print", Commands.runOnce(() -> System.out.println("THDJAKLHRUAESITYADU ILSYF")));
    NamedCommands.registerCommand("TestMe", Commands.runOnce(() -> {
      System.out.println("TestMe command executed!");
      SmartDashboard.putNumber("GGGGGGGGG", 911);
    }));

    NamedCommands.registerCommand("spin", spinFeederCommand);

    m_drivetrain.setupPathPlanner();

    // NamedCommands.registerCommand("spin", spinFeederCommand);

    configureBindings();
    m_drivetrain.printOffsets();
  }

  public Command getSpinMotorCommand() {
    return Commands.runOnce(() -> {
      System.out.println("SPIN COMMAND IS BEING CALLED!");
    });
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
        driveCommand
    // m_drivetrain.driveCommand(() -> m_joystick.getRawAxis(1) * -1,
    // () -> m_joystick.getRawAxis(0) * -1,
    // () -> m_joystick.getRawAxis(2) * -1)
    );
    
    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
    // toPoseButton.onTrue(Commands.runOnce(() -> m_drivetrain.toPose(new Pose2d(3,
    // 1, m_drivetrain.get))));
    zeroWheels.onTrue(Commands.runOnce(m_drivetrain::zeroWheels));
    inputSpin.whileTrue(spinFeederCommand);

    // Add this to configureBindings()
    Trigger testCommandTrigger = new Trigger(() -> m_joystick.getRawButton(5)); // or use another unused button
    testCommandTrigger.onTrue(Commands.runOnce(() -> {
      System.out.println("Manually triggering the TestMe command");
      Command testCommand = NamedCommands.getCommand("TestMe");
      if (testCommand != null) {
        testCommand.schedule();
      } else {
        System.out.println("ERROR: TestMe command not found in registry!");
      }
    }));
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

    return m_drivetrain.getAutonomousCommand("Scizo");
    // return m_drivetrain.getAutonomousCommand();
  }

  public void periodic(){
    // Trigger offsetButton = new Trigger(() -> m_joystick.getRawButton(5));
    // offsetButton.onTrue(Commands.runOnce(m_drivetrain::printOffsets));
    m_drivetrain.printOffsets();
  }
  public static double deadzone(double num, double deadband) {
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }

}