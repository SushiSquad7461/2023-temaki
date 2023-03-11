// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalancePID extends CommandBase {
  /** Creates a new autoBalance. */

  Swerve swerve;
  Translation2d tilt;
  PIDController pid;

  public AutoBalancePID() {
    swerve = Swerve.getInstance();
    addRequirements(swerve);
    pid = new PIDController(0.005, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("roll", getRoll());
    SmartDashboard.putNumber("pitch", getPitch());

    tilt = new Translation2d(getRoll(), -getPitch()); // TODO: switch them?? 

    pid.setSetpoint(0.0);
    double out = pid.calculate(-tilt.getNorm());



    swerve.drive(tilt.times(out), 0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 1, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (tilt.getNorm()<2); //TODO: make constants
  }

  private double getRoll() {
    return swerve.getGyro().getRoll();
  }

  private double getPitch() {
    return swerve.getGyro().getPitch();
  }
}
