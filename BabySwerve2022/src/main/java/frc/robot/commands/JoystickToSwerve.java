// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class JoystickToSwerve extends CommandBase {
  /** Creates a new JoystickToSwerve. */
  private SwerveSubsystem swerveSubsystem;
  private Joystick driver;
  private Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
  private Supplier<Boolean> fieldOrientedDrive;
  private SlewRateLimiter xLimiter, yLimiter, turnLimiter; 

  public JoystickToSwerve(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction, Supplier<Boolean> fieldOrientedDrive) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;
    this.fieldOrientedDrive = fieldOrientedDrive;

    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    addRequirements(this.swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values on sticks and deadzone them
    double xSpeed = SwerveModule.deadzone(xSpdFunction.get());
    double ySpeed = SwerveModule.deadzone(ySpdFunction.get());
    

    double turnSpeed = SwerveModule.deadzone(turnSpdFunction.get());

    // limit power
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turnSpeed = turnLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // System.out.println(xSpeed); 
    System.out.println(ySpeed); 
    // convert to chassis speed class
    swerveSubsystem.joystickToChassis(xSpeed, ySpeed, turnSpeed, fieldOrientedDrive);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
