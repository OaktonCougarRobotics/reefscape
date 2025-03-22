package frc.robot.commands;

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
    difference = target - arm.getWristPosition();
  }

  @Override
  public void initialize() {
    addRequirements(arm);
  }

  @Override
  public void execute() {
    difference = target - arm.getWristPosition();
    double kp = 0.075;
    double targetSpeed = kp * difference;
    arm.m_WristMotor.set(targetSpeed);

    System.out.println("target wrist angle:" + target);
    System.out.println("current wrist angle:" + arm.getWristPosition());
  }

  @Override
  public void end(boolean interrupted) {
    arm.m_WristMotor.set(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(difference) < 0.5;
  }
}