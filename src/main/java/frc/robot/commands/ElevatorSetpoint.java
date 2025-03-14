package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ElevatorSetpoint extends Command {
  private final TalonFX elevator;
  private double target;
  private PIDController controller = new PIDController(0.5,0,0);
  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(TalonFX elevator, double targetTurns) {
    this.elevator = elevator;
    target = targetTurns;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setPosition(Constants.BOTTOM_TURNS + target);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return elevator.getPosition().getValueAsDouble() == (Constants.BOTTOM_TURNS + target);
  }
}