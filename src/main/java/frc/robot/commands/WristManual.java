package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class WristManual extends Command {
  private TalonFX wrist;
  private double speed;

  /**
   * Constructs a wrist movement command.
   *
   * @param wrist the wrist motor object
   * @param speed    the speed we aim to run the wrist motor at
   */
  public WristManual(TalonFX wrist, double speed) {
    this.wrist = wrist;
    this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    wrist.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    wrist.set(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}