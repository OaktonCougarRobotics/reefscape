package frc.robot.commands;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Position;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmSetpointCommand extends Command{
    // Arm
    private final Arm arm;
    // PID Controllers
    private  PIDController wristPID;
    private  PIDController elevatorUpPID;
    private  PIDController elevatorDownPID;
    // PID Constants
    private final double wrist_kP = 0.75;
    private final double wrist_kI = 0.0;
    private final double wrist_kD = 0.1;
    private  double elevator_kP = 0.3;
    private  double elevator_kI = 0.0;
    private final double elevator_kD = 0.0;
    // DoubleSuppliers for Desired Arm States
    private final DoubleSupplier wristPosition;
    private final DoubleSupplier elevatorPosition;
    private final DoubleSupplier intakeVelocity;
    
    public ArmSetpointCommand(Arm arm, DoubleSupplier wristPosition, DoubleSupplier elevatorPosition, DoubleSupplier intakeVelocity){
        this.arm = arm;
        // wrist
        this.wristPID = new PIDController(wrist_kP, wrist_kI , wrist_kD);
        this.wristPosition = wristPosition;
        // elevator
        // this.elevatorUpPID = new PIDController(elevator_kP/20, elevator_kI, elevator_kD);
        this.elevatorDownPID = new PIDController(elevator_kP, elevator_kI, elevator_kD);
        // 

        var config = new TalonFXConfiguration();
        var slot0Configs = config.Slot0;
        slot0Configs.kS = 0.0; 
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 5.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 25; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        arm.getElevator().getConfigurator().apply(config);
        
        this.elevatorPosition = elevatorPosition;
        this.intakeVelocity = intakeVelocity;
    }
        public HashSet<Subsystem> getRequirements(){
        HashSet<Subsystem> req = new HashSet<>();
        req.add(arm);
        return req;
    }
    public void execute () {
        // Control Wrist
        arm.getWrist().setControl(new DutyCycleOut(wristPID.calculate(arm.getWrist().getPosition().getValueAsDouble(), wristPosition.getAsDouble())));
        // Control Elevator
        DynamicMotionMagicVoltage m_request =
            new DynamicMotionMagicVoltage(elevatorPosition.getAsDouble(), 40, 15, 4000);
        arm.getElevator().setControl(m_request);
        //     arm.getElevator().setControl(new DutyCycleOut(elevatorUpPID.calculate(arm.getElevator().getPosition().getValueAsDouble(), elevatorPosition.getAsDouble())));
        // else
        // if(elevatorPosition.getAsDouble()<arm.getElevator().getPosition().getValueAsDouble()){
        //     DynamicMotionMagicVoltage m_request =
        //         new DynamicMotionMagicVoltage(elevatorPosition.getAsDouble(), 15, 15, 4000);
        //     arm.getElevator().setControl(m_request);
        // }
        // else {
        //     arm.getElevator().setControl(new DutyCycleOut(elevatorDownPID.calculate(arm.getElevator().getPosition().getValueAsDouble(), elevatorPosition.getAsDouble())));
        // }
        // // Control Intake
            arm.getIntake().set(TalonSRXControlMode.PercentOutput, intakeVelocity.getAsDouble());
    }
}
