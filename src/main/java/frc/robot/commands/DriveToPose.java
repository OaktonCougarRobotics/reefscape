package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command{
    private Joystick joystick;
    private Drivetrain drivetrain;
  /**
   * Constructs an elevator movement command.
   *
   * @param elevator the elevator motor object
   * @param speed    the speed we aim to run the elevator motor at
   */
  public DriveToPose(Joystick joystick, Drivetrain drivetrain) {
    this.joystick = joystick;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    addRequirements(drivetrain);
    drivetrain.driveToPose(null).schedule();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveToPose(null).cancel();
  }

  @Override
  public boolean isFinished() {
    return joystick.getRawButtonReleased(2);
  }
}
