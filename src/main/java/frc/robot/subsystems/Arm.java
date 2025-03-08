package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Arm {
    private TalonFX elevator;

    public Arm(TalonFX elevator) {
        this.elevator = elevator;
    }

    public void GoToSetPoint(double targetTurns) {
        elevator.setPosition(Constants.BOTTOM_TURNS + targetTurns);
    }

}
