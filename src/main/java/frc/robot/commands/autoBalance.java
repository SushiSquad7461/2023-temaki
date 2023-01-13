// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class autoBalance extends CommandBase {
  /** Creates a new autoBalance. */

  Swerve swerve;

  double inputSensitivity;
  double controlEffort;
  double pitch;
  double roll;
  

  public autoBalance() {
    swerve = Swerve.getInstance();
    inputSensitivity = 1; // TODO: make constant or function or smth idrc
    controlEffort = - Constants.kSwerve.MAX_SPEED * 0.007;

    pitch = getPitch();
    roll = getRoll();


    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = getPitch();
    roll = getRoll();

    Translation2d movement = new Translation2d(roll, pitch).times(controlEffort); // TODO: switch them?? 

    swerve.drive(movement, 0, true, false);

    SmartDashboard.putNumber("IN AUTO BALANCE", 1);
    SmartDashboard.putNumber("Pitch", pitch);

    SmartDashboard.putNumber("Roll", roll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO: cross wheels
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(pitch)<2 && Math.abs(roll)<2); //TODO: make constants
  }

  private double getRoll() {
    return swerve.getRoll().getDegrees() * inputSensitivity;
  }

  private double getPitch() {
    return swerve.getPitch().getDegrees() * inputSensitivity;
  }



}
