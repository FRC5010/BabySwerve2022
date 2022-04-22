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
  private SwerveModule swerve0;
  private SwerveModule swerve1;
  private SwerveModule swerve2;
  private SwerveModule swerve3;

  private AHRS gyro;

  public SwerveSubsystem(SwerveModule bottomRight,SwerveModule topRight,SwerveModule bottomLeft,SwerveModule topLeft, AHRS gyro) {
    this.swerve0 = bottomRight;
    this.swerve1 = topRight;
    this.swerve2 = bottomLeft;
    this.swerve3 = topLeft;
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
    swerve0.stop();
    swerve1.stop();
    swerve2.stop();
    swerve3.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    swerve0.setDesiredState(desiredStates[0]);
    swerve1.setDesiredState(desiredStates[1]);
    swerve2.setDesiredState(desiredStates[2]);
    swerve3.setDesiredState(desiredStates[3]);
  }
}
