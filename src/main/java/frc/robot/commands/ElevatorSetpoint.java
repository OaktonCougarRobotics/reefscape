package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

public class ElevatorSetpoint extends Command {
  private final Arm arm;
  private double target;
  private double difference;
  private Trigger override;
  public MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(Arm arm, double targetAngle, Trigger override) {
    this.arm = arm;
    target = targetAngle;
    this.override = override;
  }

  @Override
  public void initialize() {
    if(!override.getAsBoolean()) {
      arm.m_ElevatorMotor.setControl(m_MotionMagicVoltage.withPosition(target));
    }
    addRequirements(arm);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.m_ElevatorMotor.set(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(difference) < 0.5 || override.getAsBoolean();
  }
}