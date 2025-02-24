package frc;

import edu.wpi.first.math.geometry.Rotation2d;

public class AT{
    private double x;
    private double y;
    private Rotation2d robotTheta;
    private Rotation2d theta;
    private double offsetX;
    private double offsetY;
    private int id;

    public AT(double x, double y, Rotation2d robotTheta, Rotation2d theta, double offSetX, double offsetY, int id)
    {
        this.x = x;
        this.y = y;
        this.robotTheta = robotTheta;
        this.theta = theta;
        this.offsetX = offSetX;
        this.offsetY = offsetY;
        this.id = id;
    } 

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Rotation2d getRobotTheta() {
        return robotTheta;
    }

    public Rotation2d getTheta() {
        return theta;
    }

    public double getOffsetX() {
        return offsetX;
    }

    public double getOffsetY() {
        return offsetY;
    }

    public int getId() {
        return id;
    }
}