// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  private SwerveModule swerve0;
  private SwerveModule swerve1;
  private SwerveModule swerve2;
  private SwerveModule swerve3;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(SwerveModule swerve0,SwerveModule swerve1,SwerveModule swerve2,SwerveModule swerve3) {
    this.swerve0 = swerve0;
    this.swerve1 = swerve1;
    this.swerve2 = swerve2;
    this.swerve3 = swerve3;
  }

  public void setSpeedMotor(double speed){
    swerve0.setSpeedMotor(speed);
    swerve1.setSpeedMotor(speed);
    swerve2.setSpeedMotor(speed);
    swerve3.setSpeedMotor(speed);
  }

  public void setAngleMotor(double speed){
    swerve0.setAngleMotor(speed);
    swerve1.setAngleMotor(speed);
    swerve2.setAngleMotor(speed);
    swerve3.setAngleMotor(speed);
  }

  public static double minMax(double input){
    double fin = input;
    if(fin > 1)
      fin = 1;
    else if(fin < -1)
      fin = 1;
    return fin;
  }

  public static double deadzone(double input){
    double fin = minMax(input);
    if(Math.abs(fin) < .08)
      fin = 0;
    return fin;
  }

  public static double scaleInputs(double input){
    double fin = deadzone(input);
    return Math.pow(fin, 1.5);
  }

  public void setAngle(){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
