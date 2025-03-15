package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ElevatorSetpoint extends Command {
  private final Arm arm;
  private double target;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /**
   * Constructs a SpinFeeder command.
   *
   * @param elevator the elevator motor object
   * @param target   the target amount of turns that the elevator aims to reach
   */
  public ElevatorSetpoint(Arm arm, double targetTurns) {
    this.arm = arm;
    target = targetTurns;
    // var slot0Configs = new Slot0Configs();
    //     slot0Configs.kP = 0.1; // An error of 1 rotation results in 2.4 V output
    //     slot0Configs.kI = 0; // no output for integrated error
    //     slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
    // arm.m_ElevatorMotor.getConfigurator().apply(slot0Configs);
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    voltageConfigs.PeakForwardVoltage = 6;
    voltageConfigs.PeakReverseVoltage = -6;
    arm.m_ElevatorMotor.getConfigurator().apply(voltageConfigs);
  }

  @Override
  public void initialize() {
    addRequirements(arm);
  }

  @Override
  public void execute() {
    // double speed = controller.calculate(arm.m_ElevatorMotor.getPosition().getValueAsDouble(), target);//Constants.BOTTOM_TURNS + target);
    // arm.m_ElevatorMotor.set(speed);
    System.out.println("Target pos in execute:" + target);
    arm.m_ElevatorMotor.setControl(m_request.withPosition(target));
    System.out.println(arm.m_ElevatorMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("sigma boy");
    arm.m_ElevatorMotor.set(0.0);
    arm.m_ElevatorMotor.setControl(new DutyCycleOut(0.0));
    System.out.println( "IsFinished Condition:  " + Math.abs(arm.m_ElevatorMotor.getPosition().getValueAsDouble() - (target)));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.m_ElevatorMotor.getPosition().getValueAsDouble() - (target)) < 0.5;//Constants.BOTTOM_TURNS + target) < 0.1;
  }
}