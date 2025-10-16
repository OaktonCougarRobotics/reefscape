package frc.robot.commands;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmSetpointCommand extends Command{
    // Arm
    private final Arm arm;
    // PID Controllers
    private final PIDController wristPID;
    private final PIDController elevatorUpPID;
    private final PIDController elevatorDownPID;
    // PID Constants
    private final double wrist_kP = 0.75;
    private final double wrist_kI = 0.0;
    private final double wrist_kD = 0.1;
    private final double elevator_kP = 0.3;
    private final double elevator_kI = 0.0;
    private final double elevator_kD = 0.0;
    // DoubleSuppliers for Desired Arm States
    private final DoubleSupplier wristPosition;
    private final DoubleSupplier elevatorPosition;
    private final DoubleSupplier intakeVelocity;
    
    public ArmSetpointCommand(Arm arm, DoubleSupplier wristPosition, DoubleSupplier elevatorPosition, DoubleSupplier intakeVelocity){
        this.arm = arm;
        this.wristPID = new PIDController(wrist_kP, wrist_kI , wrist_kD);
        this.elevatorUpPID = new PIDController(elevator_kP/20, elevator_kI, elevator_kD);
        this.elevatorDownPID = new PIDController(elevator_kP, elevator_kI, elevator_kD);

        this.wristPosition = wristPosition;
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
        if(elevatorPosition.getAsDouble()<arm.getElevator().getPosition().getValueAsDouble())
            arm.getElevator().setControl(new DutyCycleOut(elevatorUpPID.calculate(arm.getElevator().getPosition().getValueAsDouble(), elevatorPosition.getAsDouble())));
        else
            arm.getElevator().setControl(new DutyCycleOut(elevatorDownPID.calculate(arm.getElevator().getPosition().getValueAsDouble(), elevatorPosition.getAsDouble())));
        // Control Intake
            arm.getIntake().set(TalonSRXControlMode.PercentOutput, intakeVelocity.getAsDouble());
    }
}
