package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinInput extends Command{
    private TalonSRX inputMotor;
    private double amps;
    private double threshold;
    private boolean isInputing;
    private Timer timer = new Timer();
    /**
     * Constructs a Spininput command
     * 
     * @param inputMotor {@link TalonSRX } pointer towards the motor being used in the input
     * @param amps double representing at how many amps input motor runs at
     * @param threshold arbitrary value for now, should be the difference between stator voltage and commanded voltage when coral is already in place
     * @param isInputting boolean value that shows whether the command is inputting or outputing coral
     */
    public SpinInput(TalonSRX inputMotor, double amps, double threshold, boolean isInputing){
        this.inputMotor = inputMotor;
        inputMotor.setNeutralMode(NeutralMode.Brake);
        this.amps = amps;
        this.threshold = threshold;
        this.isInputing = isInputing;
    }

    //swap the -amps and amps accordingly
    @Override
    public void initialize(){
        timer.start();
        if(isInputing) {
            inputMotor.set(TalonSRXControlMode.Current, amps);    
        }
        else {
            inputMotor.set(TalonSRXControlMode.Current, -amps);
        }
    }
    @Override
    public void execute(){
        //no loop code required
    }
    @Override
    public void end(boolean interruped){
        inputMotor.set(TalonSRXControlMode.Current, 0);
    }

    public boolean isFinished(){
        if(isInputing)
            return inputMotor.getStatorCurrent()-inputMotor.getSupplyCurrent()>threshold;
        else
            return timer.get() >= 0.5;
        }
}