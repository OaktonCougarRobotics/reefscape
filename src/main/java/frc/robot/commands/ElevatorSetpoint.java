package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ElevatorSetpoint extends Command {
  private TalonFX elevator;
  private double target;

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(TalonFX elevator, double targetTurns) {
    this.elevator = elevator;
    target = targetTurns;
  }

  @Override
  public void initialize() {
    elevator.setPosition(Constants.BOTTOM_TURNS + target);
  }

  @Override
  public void execute() {
    //nothing ig
  }

  @Override
  public void end(boolean interrupted) {
    elevator.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}