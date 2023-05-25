// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    Swerve swerve;
    Translation2d tilt;
    boolean changeDir = false;
    Translation2d initialTilt;

    public AutoBalance() {
        swerve = Swerve.getInstance();
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTilt = new Translation2d(getRoll(), getPitch());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tilt = new Translation2d(getRoll(), getPitch());

        SmartDashboard.putNumber("roll", getRoll());
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("autobalance norm", tilt.getNorm());

        if(!(initialTilt.getNorm() - tilt.getNorm() > initialTilt.getNorm()/10.0) || changeDir){
            swerve.drive(tilt.times(Constants.kAutoBalance.MAX_SPEED), 0, false, false);
        } else {
            swerve.drive(new Translation2d(0, 0), 0.1, false, false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0.1, false, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return tilt.getNorm() < 0.15;
    }

    private double getRoll() {
        return swerve.getGyro().getRoll();
    }

    private double getPitch() {
        return swerve.getGyro().getPitch();
    }
}
