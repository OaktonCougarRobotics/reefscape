package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveDrive;

public class DriveCommand extends Command {
    
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;

/**
     * Constructs a DriveCommand command. Does not perform any regularization techniques
     * 
     * @param swerveDrive the swerveDrive instance
     * @param xDoubleSupplier 
     * @param yDoubleSupplier the swerveDrive instance
     * @param thetaDoubbleSupplier the swerveDrive instance
     */
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier thetaTranslationSupplier){
        this.drivetrain = drivetrain;
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.thetaTranslationSupplier = thetaTranslationSupplier;
        addRequirements(drivetrain);
    }

    public void initialize() {
        // drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(
        //   deadzone(xTranslationSupplier.getAsDouble(), Constants.Drivebase.X_DEADBAND)
        //       * drivetrain.swerveDrive.getMaximumChassisVelocity(),
        //   deadzone(YTranslationSupplier.getAsDouble(), Constants.Drivebase.Y_DEADBAND)
        //       * drivetrain.swerveDrive.getMaximumChassisVelocity(),
        //   deadzone(thetaTranslationSupplier.getAsDouble(), Constants.Drivebase.Z_DEADBAND)
        //       * drivetrain.swerveDrive.getMaximumChassisAngularVelocity()),
        //   new Translation2d());
    }

    public void execute(){
        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(
          deadzone(xTranslationSupplier.getAsDouble(), Constants.Drivebase.X_DEADBAND)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),
          deadzone(yTranslationSupplier.getAsDouble(), Constants.Drivebase.Y_DEADBAND)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),
          deadzone(thetaTranslationSupplier.getAsDouble(), Constants.Drivebase.Z_DEADBAND)
              * drivetrain.swerveDrive.getMaximumChassisAngularVelocity()),
          new Translation2d());
    }

    public void end(){
        
    } 

    public boolean isFinished(){
        
        return false;
    }
    public static double deadzone(double num, double deadband){
        if (Math.abs(num) < deadband)
      return 0.0;
    return num;
    }

}
