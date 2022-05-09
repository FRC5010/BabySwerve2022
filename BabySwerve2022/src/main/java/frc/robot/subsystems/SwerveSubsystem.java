// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private SwerveModule backRight;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule frontLeft;

  private AHRS gyro;

  public SwerveSubsystem(SwerveModule frontLeft,SwerveModule backLeft,SwerveModule frontRight,SwerveModule backRight, AHRS gyro) {
    this.backRight = backRight;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.frontLeft = frontLeft;

    this.gyro = gyro;

    new Thread(() -> {
      try{
        Thread.sleep(1000);
      }catch(Exception e){}
      zeroHeading();
    }).start();

  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    // This method will be called once per scheduler run
  }

  public void stopModules(){
    backRight.stop();
    frontRight.stop();
    backLeft.stop();
    frontLeft.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
