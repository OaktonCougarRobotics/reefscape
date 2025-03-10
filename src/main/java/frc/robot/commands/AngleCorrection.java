package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AngleCorrection extends Command {
    private Drivetrain m_drivetrain;
    private PIDController thetaController = new PIDController(.5, 0, .00001);
    private Supplier<Pose2d> targetPoseSupplier;
    private double m_thetaSpeed;

    /**
     * Constructs an AngleCorrection command
     * 
     * @param drivetrain         the drivetrain instance
     * @param targetPoseSupplier {@link java.util.function.Supplier
     *                           Supplier} that supplies the target pose of the
     *                           robot
     */
    public AngleCorrection(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_thetaSpeed = thetaController.calculate(m_drivetrain.getRotation().getRadians(),
                targetPoseSupplier.get().getRotation().getRadians());
        if (Math.abs(m_thetaSpeed) > 0.1
                && m_drivetrain.getPose().getRotation().getDegrees()
                        - targetPoseSupplier.get().getRotation().getDegrees() > 7) {
            m_drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, m_thetaSpeed));
            m_thetaSpeed = thetaController.calculate(m_drivetrain.getRotation().getRadians(),
                    targetPoseSupplier.get().getRotation().getRadians());
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.swerveDrive.drive(new ChassisSpeeds());
        thetaController.close();
    }

    public boolean isFinished() {
        
        return !(Math.abs(m_thetaSpeed) > 0.1
                && m_drivetrain.getPose().getRotation().getDegrees()
                 - targetPoseSupplier.get().getRotation().getDegrees() > 7);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
