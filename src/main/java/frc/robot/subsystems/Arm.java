package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public TalonFX m_ElevatorMotor;
    public TalonFX m_WristMotor;
    public TalonSRX m_IntakeMotor;
    

    public Arm(TalonFX elev, TalonFX wrist, TalonSRX intake) {
        this.m_ElevatorMotor = elev;
        this.m_WristMotor = wrist;
        this.m_IntakeMotor = intake;

        TalonFXConfiguration m_configsWrist = new TalonFXConfiguration();
        m_configsWrist.Slot0.kP = 18;
        m_configsWrist.Slot0.kI = 12;
        m_configsWrist.Slot0.kD = 0;
        m_configsWrist.Slot0.kG = 0.05;
        m_configsWrist.Feedback.SensorToMechanismRatio = 7;

        m_configsWrist.MotionMagic.MotionMagicCruiseVelocity = 20;
        m_configsWrist.MotionMagic.MotionMagicAcceleration = 40;
        m_configsWrist.MotionMagic.MotionMagicJerk = 400;

        m_WristMotor.getConfigurator().apply(m_configsWrist);

        TalonFXConfiguration m_configsElevator = new TalonFXConfiguration();
        m_configsElevator.Slot0.kP = 0.05;
        m_configsElevator.Slot0.kI = 0;
        m_configsElevator.Slot0.kD = 0;
        m_configsElevator.Slot0.kG = 0.05;

        m_configsElevator.MotionMagic.MotionMagicCruiseVelocity = 20;
        m_configsElevator.MotionMagic.MotionMagicAcceleration = 40;
        m_configsElevator.MotionMagic.MotionMagicJerk = 400;

        m_ElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        m_WristMotor.setNeutralMode(NeutralModeValue.Brake);
        m_IntakeMotor.setNeutralMode(NeutralMode.Brake);

        m_ElevatorMotor.setPosition(0);
        m_WristMotor.setPosition(0);
    }
    
    public double getElevatorPosition() {
        return m_ElevatorMotor.getPosition().getValueAsDouble();
    }

    public double getWristPosition() {
        return m_WristMotor.getPosition().getValueAsDouble();
    }
}
