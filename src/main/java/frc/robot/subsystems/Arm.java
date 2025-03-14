package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.robot.Constants;

public class Arm {
    private TalonFX m_ElevatorMotor;
    private SparkMax m_CoralPivotMotor;

    public Arm(TalonFX elev, SparkMax m_PivotMotor) {
        this.m_ElevatorMotor = elev;
        this.m_CoralPivotMotor = m_PivotMotor;
        // m_CoralPivotMotor = new SparkMax(1, MotorType.kBrushless);
        // SparkMaxConfig config = new SparkMaxConfig();
        // config.closedLoop.pid(1.0, 0.0, 0.0);
        // m_CoralPivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void GoToSetPoint(double targetTurns) {
        // m_ElevatorMotor.set(new VelocityDutyCycle(0.5));
        // m_ElevatorMotor.setPosition(Constants.BOTTOM_TURNS + targetTurns);
    }

    public void intake(TalonSRX m_Motor){
        // m_Motor.set(ControlMode.PercentOutput, 0.4);
    }

    
        


}
