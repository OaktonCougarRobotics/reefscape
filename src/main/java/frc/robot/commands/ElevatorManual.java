package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorManual extends Command {
  private TalonFX elevator;
  private double speed;

  /**
   * Constructs an elevator movement command.
   *
   * @param elevator the elevator motor object
   * @param speed    the speed we aim to run the elevator motor at
   */
  public ElevatorManual(TalonFX elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.set(speed);
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