package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Arm {
    // private TalonFX m_ElevatorMotor;
    // private SparkMax m_CoralPivotMotor;

    // public Arm(TalonFX elev, SparkMax m_PivotMotor) {
    //     this.m_ElevatorMotor = elev;
    //     this.m_CoralPivotMotor = m_PivotMotor;
    //     m_CoralPivotMotor = new SparkMax(1, MotorType.kBrushless);
    //     var slot0Configs = new Slot0Configs();
    //     slot0Configs.kP = -1; // An error of 1 rotation results in 2.4 V output
    //     slot0Configs.kI = -1; // no output for integrated error
    //     slot0Configs.kD = -1; // A velocity of 1 rps results in 0.1 V output

    //     m_ElevatorMotor.getConfigurator().apply(slot0Configs);

    //     SparkMaxConfig config = new SparkMaxConfig();
    //     config.closedLoop.pid(1.0, 0.0, 0.0);
    //     config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //     m_CoralPivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //     m_ElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        
    // }

    // public void GoToSetPoint(double targetTurns) {
    //     // m_ElevatorMotor.set(new VelocityDutyCycle(0.5));
    //     m_ElevatorMotor.setPosition(Constants.BOTTOM_TURNS + targetTurns);
    // }

    // public void limits(){
    //     if(m_ElevatorMotor.getPosition().getValueAsDouble() >= Constants.UPPER_LIMIT && m_ElevatorMotor.getVelocity().getValueAsDouble() > 0.0 || m_ElevatorMotor.getPosition().getValueAsDouble() <= Constants.LOWER_LIMIT && m_ElevatorMotor.getVelocity().getValueAsDouble() < 0.0){
    //         m_ElevatorMotor.set(0);
    //     }
    // }
    // public void intake(TalonSRX m_Motor){
    //     m_Motor.set(ControlMode.PercentOutput, 0.4);
    // }

    
        


}
