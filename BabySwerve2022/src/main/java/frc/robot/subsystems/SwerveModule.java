// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax driveMotor;
  private CANSparkMax turningMotor;

  private AnalogInput absoluteEncoder;

  //private SparkMaxPIDController angleSparkController;
  private SparkMaxPIDController speedSparkController;

  private ProfiledPIDController turningController;

  private int absoluteEncoderPort;
  private RelativeEncoder turningEncoder;
  private RelativeEncoder driveEncoder;

  private final boolean absouluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private double angleSetPoint;

  public SwerveModule(int driveID, int turningID ,int absEncoderPort, double radOffset, boolean reversed) {
    this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    this.turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);

    this.absoluteEncoderPort = absEncoderPort;
    absoluteEncoderOffsetRad = radOffset;
    absouluteEncoderReversed = reversed;
    
    this.turningEncoder = turningMotor.getEncoder();
    this.driveEncoder = driveMotor.getEncoder();
    this.absoluteEncoder = new AnalogInput(absEncoderPort);

    // set units drive encoder to meters and meters/sec
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    // set units turning encoder to radians and radians/sec
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // trapizoid constrants control the velocity and acceration of the motor in meters respectively
    turningController = new ProfiledPIDController(
      ModuleConstants.kPTurning, 
      ModuleConstants.kITurning, 
      ModuleConstants.kDTurning, 
      new TrapezoidProfile.Constraints(5, 2)
    );
    turningController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public void setSpeedMotor(double speed){
    driveMotor.set(speed);
  }

  public void setAngleMotor(double speed){
    turningMotor.set(speed);
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }
  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getRawEncoderVolt(){
    return absoluteEncoder.getAverageVoltage();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle *= (absouluteEncoderReversed ? -1.0 : 1.0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningController.calculate(getTurningPosition(),state.angle.getRadians()));
    SmartDashboard.putString("Swerve [" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void resetEncoders(){
    driveEncoder.setPositionConversionFactor(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  //
  // added code, might not use
  //
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

  public double getAbsRadAngle(){
    // gets voltage, then converts to radians then shifts range from (0,PI) to (-PI,PI)
    return absoluteEncoder.getAverageVoltage() * ModuleConstants.voltsToRadians - Math.PI;
  }

  public double getAbsDegreeAngle(){
    // guessing how many volts are in a rotation, between 4.8-4.9
    return absoluteEncoder.getAverageVoltage() * ModuleConstants.voltsToDegrees;
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
    SmartDashboard.putNumber("Swerve: " + absoluteEncoderPort, this.getRawEncoderVolt());
    SmartDashboard.putNumber("Swerve Angle: " + absoluteEncoderPort, this.getAbsDegreeAngle());
    double currVal = this.getRawEncoderVolt();
    if(currVal > greatestVal){
      greatestVal = currVal;
    }

    SmartDashboard.putNumber("Greatest Volt", greatestVal);
    // This method will be called once per scheduler run
  }
}
