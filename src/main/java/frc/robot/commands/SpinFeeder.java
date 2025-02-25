package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class SpinFeeder extends Command {
    private final Timer timer = new Timer();
    private final double duration = 1.0;
    private TalonSRX feederMotor;

    /**
     * Constructs a SpinFeeder command.
     *
     * @param feederMotor the motor object so the command can access and set its
     *                    values
     */
    public SpinFeeder(TalonSRX feederMotor) {
        this.feederMotor = feederMotor;
    }

    @Override
    public void initialize() {
        System.out.println("START START START START START");
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // setting motor is a basic command that does not require
        // code in the execute method to constantly run
    }

    @Override
    public void end(boolean interrupted) {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= duration;
    }
}