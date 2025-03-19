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
  public WristSetpoint(Arm arm, double targetRotations) {
    this.arm = arm;
    target = targetRotations;
    difference = arm.m_WristMotor.getPosition().getValueAsDouble() - target;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    difference = arm.m_WristMotor.getPosition().getValueAsDouble() - target;
    double kp = 0.15;
    double targetSpeed = kp * difference;
    arm.m_WristMotor.set(targetSpeed);

    System.out.println("target wrist angle:" + target);
    System.out.println("current wrist angle:" + arm.m_WristMotor.getPosition().getValueAsDouble());
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