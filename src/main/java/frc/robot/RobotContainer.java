// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AngleCorrection;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.SpinFeeder;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.io.File;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

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
  public TalonFX m_elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR);
  public TalonFX m_wristMotor = new TalonFX(Constants.WRIST_MOTOR);
  public TalonSRX m_intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
  public TalonFX m_leftClimb = new TalonFX(Constants.LEFT_CLIMB_MOTOR);
  public TalonFX m_rightClimb = new TalonFX(Constants.RIGHT_CLIMB_MOTOR);

  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  public final Arm m_Arm = new Arm(m_elevatorMotor, m_wristMotor, m_intakeMotor);

  // buttonboard object
  public final GenericHID m_buttonBoard = new GenericHID(0);
  public final GenericHID m_buttonBoard2 = new GenericHID(2);

  public final Trigger resetArmPose = new Trigger(() -> m_buttonBoard.getRawButton(2));
  public final Trigger m_l4 = new Trigger(() -> m_buttonBoard.getRawButton(4));
  public final Trigger m_l3 = new Trigger(() -> m_buttonBoard.getRawButton(5));
  public final Trigger m_l2 = new Trigger(() -> m_buttonBoard.getRawButton(6));
  public final Trigger m_l1 = new Trigger(() -> m_buttonBoard.getRawButton(7));

  // Joystick object
  public final Joystick m_joystick = new Joystick(1);

  // Triggers on the joystick
  private Trigger m_navxReset = new Trigger(() -> m_joystick.getRawButton(3));
  private Trigger m_swerveLock = new Trigger(() -> m_joystick.getRawButton(6));
  private Trigger m_toPose = new Trigger(() -> m_joystick.getRawButton(2));

  // triggers on the button board
  private Trigger m_ArmUp = new Trigger(() -> m_buttonBoard.getRawButton(Constants.ELEVATOR_UP));
  private Trigger m_ArmDown = new Trigger(() -> m_buttonBoard.getRawButton(Constants.ELEVATOR_DOWN));
  private Trigger m_WristForward = new Trigger(() -> m_buttonBoard.getRawButton(Constants.WRIST_FORWARD));
  private Trigger m_WristReverse = new Trigger(() -> m_buttonBoard.getRawButton(Constants.WRIST_REVERSE));
  private Trigger m_FlywheelIn = new Trigger(() -> m_buttonBoard.getRawButton(Constants.INTAKE_IN));
  private Trigger m_FlywheelOut = new Trigger(() -> m_buttonBoard.getRawButton(Constants.INTAKE_OUT));

  private Trigger m_ClimbLeftUp = new Trigger(() -> m_buttonBoard2.getRawButton(Constants.CLIMB_LEFT_UP));
  private Trigger m_ClimbLeftDown = new Trigger(() -> m_buttonBoard2.getRawButton(Constants.CLIMB_LEFT_DOWN));
  private Trigger m_ClimbRightUp = new Trigger(() -> m_buttonBoard2.getRawButton(Constants.CLIMB_RIGHT_UP));
  private Trigger m_ClimbRightDown = new Trigger(() -> m_buttonBoard2.getRawButton(Constants.CLIMB_RIGHT_DOWN));

  private Trigger m_toPose = new Trigger(() -> m_joystick.getRawButton(2));

  // feeder motor (anatoli)
  private TalonSRX m_feederMotor = new TalonSRX(22);
  // elevator, wrist, and flywheel

  SpinFeeder spinFeederCommand = new SpinFeeder(m_feederMotor);

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  Pose2d test = new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180)));
  DriveCommand driveCommand = new DriveCommand(
      m_drivetrain,
      () -> m_joystick.getRawAxis(1) * -1,
      () -> m_joystick.getRawAxis(0) * -1,
      () -> m_joystick.getRawAxis(2) * -1);
  AngleCorrection angleCorrection = new AngleCorrection(m_drivetrain, () -> {
    return test;
  });
  // ElevatorManual elevatorUp = new ElevatorManual(m_elevatorMotor, 0.3);
  // ElevatorManual elevatorDown = new ElevatorManual(m_elevatorMotor, -0.3);

  public RobotContainer() {
    NamedCommands.registerCommand("TestMe", Commands.runOnce(() -> {
      SmartDashboard.putNumber("testingNamedCommands", 6232025);
    }));
    // ALL THE NAMED COMMANDS; UNCOMMENT WHEN THE METHODS THEY USE ARE ACTUALLY
    // TESTED
    // NamedCommands.registerCommand("Elevators; L4", new ElevatorSetpoint(m_Arm,
    // Constants.ELEVATORS_L4));
    // NamedCommands.registerCommand("Elevators; L3", new ElevatorSetpoint(m_Arm,
    // Constants.ELEVATORS_L3));
    // NamedCommands.registerCommand("Elevators; L2", new ElevatorSetpoint(m_Arm,
    // Constants.ELEVATORS_L2));
    // NamedCommands.registerCommand("Wrist; Place L4", new WristSetpoint(m_Arm,
    // Constants.WRIST_L4));
    // NamedCommands.registerCommand("Wrist; Place L3", new WristSetpoint(m_Arm,
    // Constants.WRIST_L3));
    // NamedCommands.registerCommand("Wrist; Place L2", new WristSetpoint(m_Arm,
    // Constants.WRIST_L2));

    // NamedCommands.registerCommand("Elevators; Pickup", new
    // ElevatorSetpoint(m_Arm, Constants.ELEVATORS_PICKUP));
    // NamedCommands.registerCommand("Wrist; Pickup", new WristSetpoint(m_Arm,
    // Constants.WRIST_PICKUP));

    // NamedCommands.registerCommand("Intake", Commands.run(() ->
    // m_intakeMotor.set(ControlMode.PercentOutput, 0.2)));
    // NamedCommands.registerCommand("Outtake", Commands.run(() ->
    // m_intakeMotor.set(ControlMode.PercentOutput, -0.2)));

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

    m_ArmUp.whileTrue(new ElevatorManual(m_Arm, -0.2));
    m_ArmUp.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    m_ArmDown.whileTrue(new ElevatorManual(m_Arm, 0.2));
    m_ArmDown.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    m_WristForward.whileTrue(Commands.run(() -> {
      m_wristMotor.set(0.1);
    }));
    m_WristForward.onFalse(Commands.run(() -> {
      m_wristMotor.set(0);
    }));

    m_WristReverse.whileTrue(Commands.run(() -> {
      m_wristMotor.set(-0.1);
    }));
    m_WristReverse.onFalse(Commands.run(() -> {
      m_wristMotor.set(0);
    }));

    m_ClimbLeftUp.whileTrue(Commands.run(() -> {
      m_leftClimb.set(-0.1);
    }));
    m_ClimbLeftUp.onFalse(Commands.run(() -> {
      m_leftClimb.set(0);
    }));

    m_ClimbLeftDown.whileTrue(Commands.run(() -> {
      m_leftClimb.set(0.4);
    }));
    m_ClimbLeftDown.onFalse(Commands.run(() -> {
      m_leftClimb.set(0);
    }));

    m_ClimbRightUp.whileTrue(Commands.run(() -> {
      m_rightClimb.set(-0.1);
    }));
    m_ClimbRightUp.onFalse(Commands.run(() -> {
      m_rightClimb.set(0);
    }));

    m_ClimbRightDown.whileTrue(Commands.run(() -> {
      m_rightClimb.set(0.4);
    }));
    m_ClimbRightDown.onFalse(Commands.run(() -> {
      m_rightClimb.set(0);
    }));

    m_FlywheelIn.whileTrue(Commands.run(() -> m_intakeMotor.set(ControlMode.PercentOutput, 0.2)));
    m_FlywheelIn.onFalse(Commands.run(() -> m_intakeMotor.set(ControlMode.PercentOutput, 0)));

    m_FlywheelOut.whileTrue(Commands.run(() -> m_intakeMotor.set(ControlMode.PercentOutput, -0.2)));
    m_FlywheelOut.onFalse(Commands.run(() -> m_intakeMotor.set(ControlMode.PercentOutput, 0)));

    // FIX
    // m_l2.onTrue(new ElevatorSetpoint(m_Arm, Constants.BOTTOM_TURNS +
    // Constants.ELEVATORS_L2));
    // m_l3.onTrue(new ElevatorSetpoint(m_Arm, Constants.BOTTOM_TURNS +
    // Constants.ELEVATORS_L3));
    // m_l4.onTrue(new ElevatorSetpoint(m_Arm, Constants.BOTTOM_TURNS +
    // Constants.ELEVATORS_L4));

    m_navxReset.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
    m_swerveLock.whileTrue(Commands.run(() -> m_drivetrain.swerveDrive.lockPose()));

    // resetArmPose.onTrue(Commands.run(() -> m_elevatorMotor.setPosition(0.0)));

    // FIX: ATTEMPTS AT TOPOSE METHOD W/ CANCELLATION
    // i think this might work, but not entirely sure
    // m_toPose.onTrue(Commands.runEnd(
    // () -> {
    // m_drivetrain.driveToPose(test).execute();
    // angleCorrection.schedule();
    // },
    // () -> {
    // /* does nothing on end, maybe we schedule the normal drive command? */
    // },
    // m_drivetrain));

    m_toPose.onTrue(
      Commands.runOnce(
        ()-> {
          if(m_drivetrain.within(m_drivetrain.getPose(), test)) {
            angleCorrection.schedule();
          } else {
            m_drivetrain.driveToPose(Constants.aprilPose[m_drivetrain.closestAprilTag()].getOffestPose()).schedule();
          }
        }
      ).andThen(Commands.print(Constants.aprilPose[m_drivetrain.closestAprilTag()].getOffestPose().toString()))
    // angleCorrection
    // () -> {
    // if (!m_drivetrain.within(test, m_drivetrain.getPose())) {
    // m_drivetrain.driveToPose(test);
    // } else {
    // angleCorrection.schedule();
    // }
    // }
    // )
    );

    // m_toPose.onTrue(Commands.runOnce(() -> {m_drivetrain.driveToPose(test);}, m_drivetrain));
    // m_toPose.onTrue(Commands.runOnce(() -> {m_drivetrain.driveToPose(test).schedule();}, m_drivetrain));

    // m_toPose.onTrue(
    // Commands.runOnce(
    // () -> {
    // Command drive = m_drivetrain.driveToPose(test);
    // drive.execute();
    // if (m_drivetrain.within(m_drivetrain.getPose(), test)) {
    //  angleCorrection.schedule();
    // } else if (m_joystick.getRawAxis(1) > OperatorConstants.X_DEADBAND
    //   || m_joystick.getRawAxis(0) > OperatorConstants.X_DEADBAND
    //   || m_joystick.getRawAxis(2) > OperatorConstants.X_DEADBAND) {
    //   // CommandScheduler.getInstance().cancel(angleCorrection);
    //   // CommandScheduler.getInstance().cancel(drive);
    //   CommandScheduler.getInstance().cancelAll();
    //   // angleCorrection.cancel();
    //   // drive.cancel();
    // } // else {
    // // drive.schedule();
    // // }
    // }));
    Command driveProxy = new ProxyCommand(() -> new Command() {
      @Override
      public boolean isFinished() {
        return m_joystick.getRawButtonReleased(2);
      }

      @Override
      public void execute() {
      }

      @Override
      public void initialize(){
        m_drivetrain.driveToPose(test).schedule();
      }

      @Override
      public void end(boolean interrupted) {
        m_drivetrain.driveToPose(test).cancel();
      }
    });
    
    m_toPose.whileTrue(
      driveProxy
    );

    // () -> m_drivetrain.driveToPose(test)).andThen(angleCorrection)
    // .andThen(Commands.runOnce(() -> {
    // PIDController thetaController = new PIDController(.5, 0, .00001);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
    return m_drivetrain.getAutonomousCommand("Blue Right");
  }

  public static double deadzone(double num, double deadband) {
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }


}