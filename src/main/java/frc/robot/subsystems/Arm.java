package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public TalonFX m_ElevatorMotor;
    public TalonFX m_WristMotor;
    public TalonSRX m_IntakeMotor;
    public Counter AmMag;
    public Counter AmIndex;
    public double angleRad;

    public Arm(TalonFX elev, TalonFX wrist, TalonSRX intake) {
        this.m_ElevatorMotor = elev;
        this.m_WristMotor = wrist;
        this.m_IntakeMotor = intake;
        m_ElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        m_WristMotor.setNeutralMode(NeutralModeValue.Brake);
        m_IntakeMotor.setNeutralMode(NeutralMode.Brake);

        Constants.BOTTOM_TURNS = m_ElevatorMotor.getPosition().getValueAsDouble();
    }

    public void intake(TalonSRX m_Motor) {
        m_Motor.set(ControlMode.PercentOutput, 0.4);
    }
}
