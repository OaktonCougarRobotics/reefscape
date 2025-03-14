package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public TalonFX m_ElevatorMotor;

    public Arm(TalonFX elev) {
        this.m_ElevatorMotor = elev;
        // this.m_CoralPivotMotor = m_PivotMotor;
        // var slot0Configs = new Slot0Configs();
        // slot0Configs.kP = -1; // An error of 1 rotation results in 2.4 V output
        // slot0Configs.kI = -1; // no output for integrated error
        // slot0Configs.kD = -1; // A velocity of 1 rps results in 0.1 V output

        // m_ElevatorMotor.getConfigurator().apply(slot0Configs);

        // SparkMaxConfig config = new SparkMaxConfig();
        // config.closedLoop.pid(1.0, 0.0, 0.0);
        // config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        // m_CoralPivotMotor.configure(config, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
        m_ElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        // CurrentLimitsConfigs m_configs = new
        // CurrentLimitsConfigs().withStatorCurrentLimit(20);
        // m_ElevatorMotor.setControl(new VelocityDutyCycle(1));
        // m_ElevatorMotor.getConfigurator().apply(m_configs);
    }

    public void GoToSetPoint(double targetTurns) {
        // m_ElevatorMotor.set(new VelocityDutyCycle(0.5));
        m_ElevatorMotor.setPosition(Constants.BOTTOM_TURNS + targetTurns);
    }

    public void intake(TalonSRX m_Motor) {
        m_Motor.set(ControlMode.PercentOutput, 0.4);
    }

}
