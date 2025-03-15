package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix6.configs.VoltageConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class WristSetpoint extends Command {
  private final Arm arm;
  private double target;
  public Counter AmMag;   
  // This counter counts the index pulses (revolutions)
  public Counter AmIndex;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  double angleRAD;

  /**
   * Constructs a SpinFeeder command.
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public WristSetpoint(Arm arm, double targetAngle) {
    this.arm = arm;
    target = targetAngle;
  }

  @Override
  public void initialize() {
        AmMag = new Counter(0); 
		AmIndex = new Counter(1);
		//Set Semi-Period Mode in order to Measure the Pulse Width
		AmMag.setSemiPeriodMode(true);
  }

  @Override
  public void execute() {
    	double value = AmMag.getPeriod();
		// SmartDashboard.putNumber("Rotations", AmIndex.get());
		// SmartDashboard.putNumber("Intermediate", value);
		// The 9.73e-4 is the total period of the PWM output on the am-3749
		// The value will then be divided by the period to get duty cycle.
		// This is converted to degrees and Radians
		//double angleDEG = (value/9.739499999999999E-4)*361 -1;
		angleRAD = (value/9.739499999999999E-4)*2*(Math.PI) ;
		// SmartDashboard.putNumber("Angle in Degrees", angleDEG);
		// SmartDashboard.putNumber("Angle in Radians", angleRAD);
    System.out.println("Target pos in execute:" + target);
    arm.m_WristMotor.set(ControlMode.Position, angleRAD);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("sigma boy");
    arm.m_WristMotor.set(ControlMode.PercentOutput, 0.0);
    System.out.println( "IsFinished Condition:  " + Math.abs(arm.m_ElevatorMotor.getPosition().getValueAsDouble() - (target)));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(angleRAD - (target)) < 0.5;//Constants.BOTTOM_TURNS + target) < 0.1;
  }
}