package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

public class SpinFeeder extends Command{
    private TalonSRX feederMotor;
    /**
     * Constructs a SpinFeeder command.
     *
     * @param feederMotor the motor object so the command can access and set its values
     */
    public SpinFeeder(TalonSRX feederMotor){
        this.feederMotor = feederMotor;
    }
    @Override
    public void initialize(){
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
    }
    
    @Override
    public void execute(){
        // setting motor is a basic command that does not require
        // code in the execute method to constantly run
    }

    @Override
    public void end(boolean interrupted){
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}