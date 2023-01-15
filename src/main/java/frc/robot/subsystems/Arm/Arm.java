package frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase{
    
    public abstract double getPosition();

    public abstract double getLeftMotorVelocity();

    public abstract double getRightMotorVelocity();

    public abstract void runArm(double degrees);

    public abstract void stopArm();

    public abstract void setPosition(double degrees);

    public abstract void resetArm();
}
