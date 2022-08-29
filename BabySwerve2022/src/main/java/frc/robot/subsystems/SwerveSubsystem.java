// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));
  private boolean ready = false;

  public SwerveSubsystem(SwerveModule frontLeft,SwerveModule frontRight,SwerveModule backLeft,SwerveModule backRight, AHRS gyro) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.gyro = gyro;

    

    new Thread(() -> {
      try{
        Thread.sleep(1000);
      }catch(Exception e){}
      zeroHeading();
    }).start();

  }

  public void zeroHeading(){
    System.out.println("------ Zeroing the heading -----");
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose2d(){
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(pose, pose.getRotation());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    // This method will be called once per scheduler run
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    boolean isReady = true;
    isReady &= frontLeft.setDesiredState(desiredStates[0], ready);
    isReady &= frontRight.setDesiredState(desiredStates[1], ready);
    isReady &= backLeft.setDesiredState(desiredStates[2], ready);
    isReady &= backRight.setDesiredState(desiredStates[3], ready);
    ready = isReady;
  }

  @Override
  public void simulationPeriodic() {
    frontLeft.simulationPeriodic();
    frontRight.simulationPeriodic();
    backLeft.simulationPeriodic();
    backRight.simulationPeriodic();
  }
}
