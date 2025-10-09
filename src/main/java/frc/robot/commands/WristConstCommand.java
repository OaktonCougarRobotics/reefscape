package frc.robot.commands;

import java.util.HashSet;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class WristConstCommand extends Command{
    private Arm arm;
    private PIDController pid;
    private double kP = .7;
    private double kI = 0.;
    private double kD = 0.;
    private double position;

    public WristConstCommand(Arm arm, double position){
        this.arm = arm;
        pid = new PIDController(kP, kI , kD);
        this.position = position;
    }
        public HashSet<Subsystem> getRequirements(){
        HashSet<Subsystem> req = new HashSet<>();
        req.add(arm);
        return req;
    }
    public void execute () {
        arm.getWrist().setControl(new DutyCycleOut(pid.calculate(arm.getWrist().getPosition().getValueAsDouble(), position)));
    }
}
