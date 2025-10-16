package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Arm extends SubsystemBase {
//     static int REPLACEME = 59;

//     private TalonSRX m_intakeMotor = new TalonSRX(-2);
//     private TalonFX m_elevatorMotor = new TalonFX(-2);
//     private TalonFX m_wristMotor = new TalonFX(-2);
//     private ElevatorState m_wantedElevatorState = ElevatorState.START;
//     private boolean elevatorInIntermediateState = false;
//     private WristState m_wantedWristState = WristState.START;
//     private boolean wristInIntermediateState = false;

// //     //position variable represents the motor position for specific setpoint
//     public enum ElevatorState{
//         START(REPLACEME),
//         INTAKE(REPLACEME),
//         L1(REPLACEME),
//         L2(REPLACEME),
//         L3(REPLACEME),
//         L4(REPLACEME);
//         private double position;
//         ElevatorState(double position){
//             this.position = position;
//         }
//         public double getPosition(){
//             return position;
//         }
//     }
//     //angle variable for the angle of the wrist for specific state
//     public enum WristState{
//         START(REPLACEME),
//         INTAKE(REPLACEME),
//         L1(REPLACEME),
//         L2(REPLACEME),
//         L3(REPLACEME),
//         L4(REPLACEME);
//         private double angle;
//         WristState(double angle){
//             this.angle = angle;
//         }
//         public double getAngle(){
//             return angle;
//         }
//     }
//     //proportionOut variable represents the power output for inputMotor (duty cycle)
//     public enum IntakeState{
//         INTAKE(REPLACEME),
//         OUTTAKE(REPLACEME),
//         NUETRAL(0);
//         private double proportionOut;
//         IntakeState(double proportionOut){
//             this.proportionOut = proportionOut;
//         }
//         public double getProportionOut(){
//             return proportionOut;
//         }
//     }
//         public Arm() {
//         m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
//         m_wristMotor.setNeutralMode(NeutralModeValue.Brake);
//         m_intakeMotor.setNeutralMode(NeutralMode.Brake);


//     }
//     public double getElevatorPosition() {
//         return m_elevatorMotor.getPosition().getValue().baseUnitMagnitude(); // must figure out our own way to interpret raw number
//     }
//     public double getWristAngle() {
//         return m_wristMotor.getPosition().getValue().baseUnitMagnitude(); // pretty sure this will always between 0 and 1/-1
//     }
//     public void setWantedElevatorState(ElevatorState state) {
//         m_wantedElevatorState = state;
//     }
    
//     public void periodic() {

//     }
// }

public class Arm extends SubsystemBase {
    private TalonSRX m_intake;
    private TalonFX m_wrist;
    private TalonFX m_elevator;
    public Arm (int wristID, int elevatorID, int intakeID){
        m_intake = new TalonSRX(intakeID);
        m_intake.setNeutralMode(NeutralMode.Brake);
        m_wrist = new TalonFX(wristID);
        m_wrist.setPosition(0);
        m_elevator = new TalonFX(elevatorID);
        m_elevator.setPosition(0);
    }
    public TalonFX getWrist(){
        return m_wrist;
    }
    public TalonFX getElevator(){
        return m_elevator;
    }
    public TalonSRX getIntake(){
        return m_intake;
    }
}
