// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.FRC5010.GenericGyro;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private SwerveModule backRight;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule frontLeft; 


  private GenericGyro gyro;
  private boolean ready = false;
  private final SwerveDriveOdometry odometer;  
  private final Pose2d pose2d = new Pose2d(); 
  private SwerveModulePosition[] modulePositions; 

  public SwerveSubsystem(SwerveModule frontLeft,SwerveModule frontRight,SwerveModule backLeft,SwerveModule backRight, GenericGyro gyro) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
    this.gyro = gyro;


    modulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.frontLeft.getAbsoluteEncoderRad()))), 
      new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.frontRight.getAbsoluteEncoderRad()))), 
      new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.backLeft.getAbsoluteEncoderRad()))), 
      new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.backRight.getAbsoluteEncoderRad())))
    };
  
    odometer = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(), modulePositions);

    // = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), null);
  
    new Thread(() -> {
      try{
        Thread.sleep(1000);
      }catch(Exception e){}
      zeroHeading();
    }).start();

  }
  
  public SwerveModulePosition[] getModulePositions(){
    return this.modulePositions; 
  }
  
  public void zeroHeading(){
    // System.out.println("------ Zeroing the heading -----");
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

  public void joystickToChassis(double xSpeed, double ySpeed, double turnSpeed, Supplier<Boolean> fieldOrientedDrive){

    ChassisSpeeds chassisSpeeds;
    // 
    if(fieldOrientedDrive.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, 
        ySpeed, 
        turnSpeed, 
        getRotation2d()
      );
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turnSpeed);
    }

    // convert chassis speed into modules speeds
    SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

    // output each module speed into subsystem
    setModuleStates(moduleStates);
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), modulePositions, pose2d);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
    odometer.update(getRotation2d(), modulePositions);
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
