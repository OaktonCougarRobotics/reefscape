package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {

    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;
    private double rotateMod = 0;
    private double translationMod = 0.3;

    /**
     * Constructs a DriveCommand command
     * 
     * @param swerveDrive          the swerveDrive instance
     * @param xDoubleSupplier      {@link java.util.function.DoubleSupplier
     *                             DoubleSupplier} that supplies the double value
     *                             between [-1,1] for joystick x translation
     * @param yDoubleSupplier      {@link java.util.function.DoubleSupplier
     *                             DoubleSupplier} that supplies the double value
     *                             between [-1,1] for joystick y translation
     * @param thetaDoubbleSupplier {@link java.util.function.DoubleSupplier
     *                             DoubleSupplier} that supplies the double value
     *                             between [-1,1] for joystick theta translation
     */
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier,
            DoubleSupplier thetaTranslationSupplier) {
        this.drivetrain = drivetrain;
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.thetaTranslationSupplier = thetaTranslationSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(
                cubicMod(deadzone(xTranslationSupplier.getAsDouble(), Constants.OperatorConstants.X_DEADBAND)
                        * drivetrain.swerveDrive.getMaximumChassisVelocity(), translationMod),
                cubicMod(deadzone(yTranslationSupplier.getAsDouble(), Constants.OperatorConstants.Y_DEADBAND)
                        * drivetrain.swerveDrive.getMaximumChassisVelocity(), translationMod),
                cubicMod(deadzone(thetaTranslationSupplier.getAsDouble(), Constants.OperatorConstants.Z_DEADBAND)
                        * drivetrain.swerveDrive.getMaximumChassisAngularVelocity(), rotateMod)),
                new Translation2d());
    }

    @Override
    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    public static double deadzone(double num, double deadband) {
        if (Math.abs(num) < deadband)
            return 0.0;
        return num;
    }

    public static double cubicMod(double in, double cm) {
        return cm * Math.pow(in, 3) + (1 - cm) * in;
    }
}
