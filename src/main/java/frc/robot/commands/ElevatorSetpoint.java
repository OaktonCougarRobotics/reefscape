package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ElevatorSetpoint extends Command {
  private final Arm arm;
  private double target;
  private double difference;
  private double targetSpeed;

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(Arm arm, double targetRotations) {
    this.arm = arm;
    target = targetRotations;
    difference = arm.m_ElevatorMotor.getPosition().getValueAsDouble() - target;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    difference = target - arm.m_ElevatorMotor.getPosition().getValueAsDouble();
    double kp = 0.001;
	targetSpeed = kp * difference;
	arm.m_ElevatorMotor.set(targetSpeed);

    System.out.println("difference in elevator positions:" + difference);
    System.out.println("current elevator position:" + arm.m_ElevatorMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
	  arm.m_ElevatorMotor.set(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetSpeed) < 0.0;
  }
}