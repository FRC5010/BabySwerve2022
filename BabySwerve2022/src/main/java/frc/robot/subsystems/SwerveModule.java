// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  private final boolean drivingEncoderReversed;
  private final boolean absoluteEncoderReversed;
  private final boolean turningEncoderReversed;
  private final String moduleKey;

  private double angleSetPoint;

  private MechanismRoot2d simRoot;
  private final MechanismLigament2d simDial;
  private final MechanismLigament2d absSimDial;

  public SwerveModule(int driveID, int turningID, int absEncoderPort, String radOffsetKey, 
    boolean driveReversed, boolean turningReversed, MechanismRoot2d simRoot) {
    
    this.absoluteEncoderPort = absEncoderPort;
    moduleKey = radOffsetKey;
    absoluteEncoderReversed = turningReversed;
    turningEncoderReversed = turningReversed;
    drivingEncoderReversed = driveReversed;
        
    this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    driveMotor.setInverted(drivingEncoderReversed);
    this.turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);
    turningMotor.setInverted(turningEncoderReversed);

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
      new TrapezoidProfile.Constraints(40, 40)
    );

    turningController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

    this.simRoot = simRoot;
    simDial = simRoot.append(
      new MechanismLigament2d(moduleKey, 10.0, 0.0, 6.0, new Color8Bit(Color.kYellow)));
    absSimDial = simRoot.append(
      new MechanismLigament2d(moduleKey+"Abs", 10, 0, 6, new Color8Bit(Color.kBlue)));
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
    angle -= Preferences.getDouble(moduleKey, 0);
    return angle *= (absoluteEncoderReversed ? -1.0 : 1.0);
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
    double turnPow = turningController.calculate(getTurningPosition(),state.angle.getRadians());
    // getTurningPosition()
    // adding ks to get swerve moving
    turningMotor.set(turnPow + (Math.signum(turnPow) * ModuleConstants.kS));
    SmartDashboard.putString("Swerve [" + absoluteEncoder.getChannel() + "] state", 
      "Angle: " + state.angle.getDegrees() + " Speed m/s: " + state.speedMetersPerSecond);
  }

  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
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

  double greatestVal = 0;
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Ang: " + absoluteEncoderPort, Units.radiansToDegrees(getTurningPosition()));
    SmartDashboard.putNumber("Abs Angle: " + absoluteEncoderPort, Units.radiansToDegrees(getAbsoluteEncoderRad()));
    SmartDashboard.putNumber("Abs Rads: " + absoluteEncoderPort, getAbsoluteEncoderRad());
    // This method will be called once per scheduler run
  }
}
