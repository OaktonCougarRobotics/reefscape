// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AngleCorrection;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.ElevatorSetpoint;
import frc.robot.commands.SpinFeeder;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Counter;

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
  public TalonSRX m_wristMotor = new TalonSRX(Constants.WRIST_MOTOR);
  public TalonSRX m_intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR);
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  public final Arm m_Arm = new Arm(m_elevatorMotor, m_wristMotor, m_intakeMotor);
  public Counter AmMag;
  // This counter counts the index pulses (revolutions)
  public Counter AmIndex;
  double angleRAD;

  // buttonboard object
  public final GenericHID m_buttonBoard = new GenericHID(0);

  // public final Joystick m_buttonBoard = new Joystick(x);
  // // button board buttons
  // // four extra buttons not declared
  // public final Trigger m_climbLeft = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // public final Trigger m_climbRight = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // // manual control button
  // public final Trigger m_manualWrist = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // public final Trigger m_manualElevatorLift = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // // algae button
  // public final Trigger m_algae = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // public final Trigger m_intakeSwitch = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // // different levels
  // public final Trigger m_startingArmPos = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  // public final Trigger m_coralStationPos = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));
  public final Trigger resetArmPose = new Trigger(() -> m_buttonBoard.getRawButton(2));
  public final Trigger m_l4 = new Trigger(() -> m_buttonBoard.getRawButton(4));
  public final Trigger m_l3 = new Trigger(() -> m_buttonBoard.getRawButton(5));
  public final Trigger m_l2 = new Trigger(() -> m_buttonBoard.getRawButton(6));
  public final Trigger m_l1 = new Trigger(() -> m_buttonBoard.getRawButton(7));
  // // vision button
  // public final Trigger m_horatioMagic = new Trigger(() ->
  // m_buttonBoard.getRawButton(x));

  // Joystick object
  public final Joystick m_joystick = new Joystick(1);

  // Triggers on the joystick
  private Trigger m_navxReset = new Trigger(() -> m_joystick.getRawButton(3));
  // private Trigger m_setTop = new Trigger(() -> m_joystick.getRawButton(2)); //
  // Work in Progress - Horatio
  // private Trigger m_setBottom = new Trigger(() -> m_joystick.getRawButton(1));
  // // Work in Progress - Horatio

  // triggers on the button board
  private Trigger m_ArmUp = new Trigger(() -> m_buttonBoard.getRawButton(Constants.ELEVATOR_UP));
  // private Trigger m_ArmDown = new Trigger(() -> m_buttonBoard.getRawButton(Constants.ELEVAATOR_DOWN));
  private Trigger m_WristForward = new Trigger(() -> m_buttonBoard.getRawButton(Constants.WRIST_FORWARD));
  private Trigger m_WristReverse = new Trigger(() -> m_buttonBoard.getRawButton(Constants.WRIST_REVERSE));

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
    NamedCommands.registerCommand("spin", spinFeederCommand);

    m_drivetrain.setupPathPlanner();
    configureBindings();
    AmMag = new Counter(0);
    AmIndex = new Counter(1);
    // Set Semi-Period Mode in order to Measure the Pulse Width
    AmMag.setSemiPeriodMode(true);
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

    // m_ArmUp.whileTrue(Commands.run(() -> {
    // if (m_elevatorMotor.getPosition().getValueAsDouble() > Constants.UPPER_LIMIT)
    // {
    // m_elevatorMotor.set(-0.15);
    // } else {
    // m_elevatorMotor.set(0);
    // }
    // }));
    // m_ArmUp.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    // m_ArmDown.whileTrue(Commands.run(() -> {
    // if (m_elevatorMotor.getPosition().getValueAsDouble() < Constants.LOWER_LIMIT)
    // {
    // m_elevatorMotor.set(0.15);
    // } else {
    // m_elevatorMotor.set(0);
    // }
    // }));
    // m_ArmDown.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    m_ArmUp.whileTrue(new ElevatorManual(m_Arm, -0.2));
    m_ArmUp.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    // m_ArmDown.whileTrue(new ElevatorManual(m_Arm, 0.2));
    // m_ArmDown.onFalse(Commands.run(() -> m_elevatorMotor.set(0)));

    m_WristForward.whileTrue(Commands.run(() -> m_wristMotor.set(ControlMode.PercentOutput, 0.2)));
    m_WristForward.onFalse(Commands.run(() -> m_wristMotor.set(ControlMode.PercentOutput, 0)));

    m_WristReverse.whileTrue(Commands.run(() -> m_wristMotor.set(ControlMode.PercentOutput, -0.2)));
    m_WristReverse.onFalse(Commands.run(() -> m_wristMotor.set(ControlMode.PercentOutput, 0)));

    // FIX
    // m_l2.onTrue(new ElevatorSetpoint(m_Arm, Constants.LOW_TARGET),
    // Commands.run(() -> execute(Constants.LOW_TARGET_WRIST)));
    // m_l3.onTrue(new ElevatorSetpoint(m_Arm, Constants.MID_TARGET),
    // Commands.run(() -> execute(Constants.MID_TARGET_WRIST)));
    // // m_l3.whileTrue(Commands.run(() -> {
    // // .m_ElevatorMotor.setControl(m_request.withPosition(Constants.MID_TARGET));
    // // }));
    // m_l4.onTrue(new ElevatorSetpoint(m_Arm, Constants.HIGH_TARGET),
    // Commands.run(() -> execute(Constants.HIGH_TARGET_WRIST)));

    m_navxReset.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));

    resetArmPose.onTrue(Commands.run(() -> m_elevatorMotor.setPosition(0.0)));

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

    // m_toPose.whileTrue(
    // Commands.runOnce(
    // ()->{
    // if(m_drivetrain.within(m_drivetrain.getPose(), test))
    // angleCorrection.schedule();
    // else
    // m_drivetrain.driveToPose(test);
    // }
    // )
    // // angleCorrection
    // // () -> {
    // // if (!m_drivetrain.within(test, m_drivetrain.getPose())) {
    // // m_drivetrain.driveToPose(test);
    // // } else {
    // // angleCorrection.schedule();
    // // }
    // // }
    // // )
    // );

    // m_toPose.onTrue(
    // Commands.runOnce(
    // () -> {
    // Command drive = m_drivetrain.driveToPose(test);
    // drive.execute();
    // if (m_drivetrain.within(m_drivetrain.getPose(), test)) {
    // angleCorrection.schedule();
    // } else if (m_joystick.getRawAxis(1) > OperatorConstants.X_DEADBAND
    // || m_joystick.getRawAxis(0) > OperatorConstants.X_DEADBAND
    // || m_joystick.getRawAxis(2) > OperatorConstants.X_DEADBAND) {
    // // CommandScheduler.getInstance().cancel(angleCorrection);
    // // CommandScheduler.getInstance().cancel(drive);
    // CommandScheduler.getInstance().cancelAll();
    // // angleCorrection.cancel();
    // // drive.cancel();
    // } // else {
    // // drive.schedule();
    // // }
    // }));

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
    return m_drivetrain.getAutonomousCommand("Blue rightest StartPos");
  }

  public static double deadzone(double num, double deadband) {
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }

  public void limits() {
    if ((m_Arm.m_ElevatorMotor.getPosition().getValueAsDouble() <= Constants.ARM_HIGH
        && m_Arm.m_ElevatorMotor.getVelocity().getValueAsDouble() < 0.0) ||
        (m_Arm.m_ElevatorMotor.getPosition().getValueAsDouble() >= -10
            && m_Arm.m_ElevatorMotor.getVelocity().getValueAsDouble() > 0.0)) {
      m_Arm.m_ElevatorMotor.set(0);
    }
  }

  public double calcAngle() {
    double value = AmMag.getPeriod();
    // SmartDashboard.putNumber("Rotations", AmIndex.get());
    // SmartDashboard.putNumber("Intermediate", value);
    // The 9.73e-4 is the total period of the PWM output on the am-3749
    // The value will then be divided by the period to get duty cycle.
    // This is converted to degrees and Radians
    // double angleDEG = (value/9.739499999999999E-4)*361 -1;
    angleRAD = (value / 9.739499999999999E-4) * 2 * (Math.PI);
    // SmartDashboard.putNumber("Angle in Degrees", angleDEG);
    // SmartDashboard.putNumber("Angle in Radians", angleRAD);
    return angleRAD;
  }

  public void execute(double targetAngle) {
    m_Arm.m_WristMotor.set(ControlMode.Position, targetAngle);
  }
}