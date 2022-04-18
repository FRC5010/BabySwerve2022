// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax speedMotor;
  private CANSparkMax angleMotor;
  private AnalogInput swerveEncoder;
  private SparkMaxPIDController anglePid;
  private int encoderPort;
  private RelativeEncoder angleEncoder;
  private RelativeEncoder speedEncoder;

  private final boolean absouluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private double angleSetPoint;

  public SwerveModule(CANSparkMax speedMotor, CANSparkMax angleMotor ,int encoderPort, double radOffset, boolean reversed) {
    this.speedMotor = speedMotor;
    this.angleMotor = angleMotor;
    this.encoderPort = encoderPort;
    absoluteEncoderOffsetRad = radOffset;
    absouluteEncoderReversed = reversed;
    
    this.angleEncoder = angleMotor.getEncoder();
    this.speedEncoder = speedMotor.getEncoder();
    this.swerveEncoder = new AnalogInput(encoderPort);


    this.anglePid = angleMotor.getPIDController();
    anglePid.setP(0);
    anglePid.setI(0);
    anglePid.setD(0);
    
    angleEncoder.setPosition(0);
    
  }

  public void pidAngle(){
    anglePid.setReference(angleSetPoint, CANSparkMax.ControlType.kPosition);
  }

  public void setSpeedMotor(double speed){
    speedMotor.set(speed);
  }

  public void setAngleMotor(double speed){
    angleMotor.set(speed);
  }

  public double getRawEncoderVolt(){
    return swerveEncoder.getAverageVoltage();
  }

  public double getAbsDegreeAngle(){
    // guessing how many volts are in a rotation, between 4.8-4.9
    return swerveEncoder.getAverageVoltage() * ModuleConstants.voltsToDegrees;
  }

  public double getAbsRadAngle(){
    return swerveEncoder.getAverageVoltage() * ModuleConstants.voltsToRadians;
  }

  public static double continuousMinMax(double input){
    if(input > Math.PI){
      input -= Math.PI;
    }else if(input < -Math.PI){
      input += Math.PI;
    }
    return input;
  }

  double greatestVal = 0;
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swerve: " + encoderPort, this.getRawEncoderVolt());
    SmartDashboard.putNumber("Swerve Angle: " + encoderPort, this.getAbsDegreeAngle());
    double currVal = this.getRawEncoderVolt();
    if(currVal > greatestVal){
      greatestVal = currVal;
    }

    SmartDashboard.putNumber("Greatest Volt", greatestVal);
    // This method will be called once per scheduler run
  }
}
