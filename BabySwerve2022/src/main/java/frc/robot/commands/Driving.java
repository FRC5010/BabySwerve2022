// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.SwerveModule;

public class Driving extends CommandBase {
  /** Creates a new Driving. */
  DriveTrainSubsystem driveTrainSubsystem;
  Joystick driver;
  public Driving(DriveTrainSubsystem driveTrainSubsystem, Joystick driver) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.driver = driver;
    addRequirements(driveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPow = DriveTrainSubsystem.deadzone(-driver.getRawAxis(1));
    double rightPow = DriveTrainSubsystem.deadzone(driver.getRawAxis(4));

    driveTrainSubsystem.setSpeedMotor(leftPow);
    driveTrainSubsystem.setAngleMotor(rightPow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
