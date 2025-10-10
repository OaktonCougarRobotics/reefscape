package frc.robot.commands;

import java.util.HashSet;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class WristConstCommand extends Command{
    private Arm arm;
    private PIDController wristPID;
    private PIDController elevatorPID;
    private double wrist_kP = 2.3;
    private double wrist_kI = 0.0;
    private double wrist_kD = 0.1;
    // private double elevator_kP = 0;
    // private double elevator_kI = 0.0;
    // private double elevator_kD = 0.0;
    private double wristPosition;
    private double elevatorPosition;
    
    public WristConstCommand(Arm arm, double wristPosition, double elevatorPosition){
        this.arm = arm;
        wristPID = new PIDController(wrist_kP, wrist_kI , wrist_kD);
        // elevatorPID = new PIDController(elevator_kP, elevator_kI, elevator_kD);
        this.wristPosition = wristPosition;
        this.elevatorPosition = elevatorPosition;
    }
        public HashSet<Subsystem> getRequirements(){
        HashSet<Subsystem> req = new HashSet<>();
        req.add(arm);
        return req;
    }
    public void execute () {
        arm.getWrist().setControl(new DutyCycleOut(wristPID.calculate(arm.getWrist().getPosition().getValueAsDouble(), wristPosition)));
        // arm.getElevator().setControl(new DutyCycleOut(elevatorPID.calculate(arm.getElevator().getPosition().getValueAsDouble(), elevatorPosition)));

    }
}
// up -> wrist move -> output