package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ElevatorManual extends Command {
  private Arm arm;
  private double speed;
  /**
   * Constructs an elevator movement command.
   *
   * @param elevator the elevator motor object
   * @param speed    the speed we aim to run the elevator motor at
   */
  public ElevatorManual(Arm arm, double speed) {
    this.speed = speed;
    this.arm = arm;
  }

  @Override
  public void initialize() {
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.m_ElevatorMotor.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    arm.m_ElevatorMotor.set(0.0);
  }

  // @Override
  // public boolean isFinished() {
  //   // return arm.m_ElevatorMotor.getPosition().getValueAsDouble() < Constants.UPPER_LIMIT ||
  //   // arm.m_ElevatorMotor.getPosition().getValueAsDouble() > Constants.LOWER_LIMIT;
  // }
}