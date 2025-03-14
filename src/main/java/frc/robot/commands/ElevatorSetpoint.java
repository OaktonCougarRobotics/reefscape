package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ElevatorSetpoint extends Command {
  private final Arm arm;
  private double target;
  private PIDController controller = new PIDController(0.5, 0, 0);

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(Arm arm, double targetTurns) {
    this.arm = arm;
    target = targetTurns;
  }

  @Override
  public void initialize() {
    addRequirements(arm);
  }

  @Override
  public void execute() {
    double speed = controller.calculate(arm.m_ElevatorMotor.getPosition().getValueAsDouble(),
        target);//Constants.BOTTOM_TURNS + target);
    arm.m_ElevatorMotor.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    arm.m_ElevatorMotor.set(0.0);
    controller.close();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.m_ElevatorMotor.getPosition().getValueAsDouble() - (target)) < 0.1;//Constants.BOTTOM_TURNS + target) < 0.1;
  }
}