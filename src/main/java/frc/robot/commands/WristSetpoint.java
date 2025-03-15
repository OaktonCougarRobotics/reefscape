package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class WristSetpoint extends Command {
  private final Arm arm;
  private double target;
  private double difference;

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public WristSetpoint(Arm arm, double targetAngle) {
    this.arm = arm;
    target = targetAngle;
    difference = arm.calcAngle() - target;
  }

  @Override
  public void initialize() {
    addRequirements(arm);
  }

  @Override
  public void execute() {
    difference = arm.calcAngle() - target;
    double kp = 0.15;
	  double targetSpeed = kp * difference;
	  arm.m_WristMotor.set(ControlMode.PercentOutput, targetSpeed);

    System.out.println("target wrist angle:" + target);
    System.out.println("current wrist angle:" + arm.calcAngle());
  }

  @Override
  public void end(boolean interrupted) {
	  arm.m_WristMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(difference) < 0.5;
  }
}