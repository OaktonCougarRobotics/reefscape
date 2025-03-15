package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ElevatorSetpoint extends Command {
  private final Arm arm;
  private double target;
  private double difference;
  private double targetSpeed;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(Arm arm, double targetAngle) {
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
    difference = arm.m_ElevatorMotor.getPosition().getValueAsDouble() - target;
    double kp = 0.15;
	targetSpeed = kp * difference;
	arm.m_ElevatorMotor.set(targetSpeed);

    System.out.println("target elevator position:" + target);
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