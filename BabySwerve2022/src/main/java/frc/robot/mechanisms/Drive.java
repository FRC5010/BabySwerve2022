// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DriveConstants;
import frc.robot.FRC5010.DrivetrainPoseEstimator;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.Vision.VisionPhotonCamera;
import frc.robot.commands.JoystickToSwerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Drive {
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;
    private SwerveSubsystem swerveSubsystem;
    static GenericGyro gyro;
    private Button zeroHeading;
    private Button resetEncoders;
    private final Mechanism2d mech2dVisual = new Mechanism2d(60, 60);
    private final MechanismRoot2d frontLeftVisPt = mech2dVisual.getRoot(DriveConstants.kFrontLeftKey, 45, 15);
    private final MechanismRoot2d frontRightVisPt = mech2dVisual.getRoot(DriveConstants.kFrontRightKey, 45, 45);
    private final MechanismRoot2d backLeftVisPt = mech2dVisual.getRoot(DriveConstants.kBackLeftKey, 15, 15);
    private final MechanismRoot2d backRightVisPt = mech2dVisual.getRoot(DriveConstants.kBackRightKey, 15, 45);
    private DrivetrainPoseEstimator drivetrainPoseEstimator;        
    private VisionPhotonCamera vision;

    public Drive(GenericGyro gyro){

        if(!Preferences.containsKey(DriveConstants.kFrontLeftKey)){
            Preferences.setDouble(DriveConstants.kFrontLeftKey, DriveConstants.kFrontLeftAbsoluteOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kFrontRightKey)){
            Preferences.setDouble(DriveConstants.kFrontRightKey, DriveConstants.kFrontRightAbsoluteOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kBackLeftKey)){
            Preferences.setDouble(DriveConstants.kBackLeftKey, DriveConstants.kBackLeftAbsoluteOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kBackRightKey)){
            Preferences.setDouble(DriveConstants.kBackRightKey, DriveConstants.kBackRightAbsoluteOffsetRad);    
        }
        SmartDashboard.putData("Swerve Sim", mech2dVisual);

        this.gyro = gyro;
        Joystick driver = new Joystick(0);
        frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
            DriveConstants.kFrontLeftTurningMotorPort, 
            DriveConstants.kFrontLeftAbsolutePort, 
            DriveConstants.kFrontLeftKey, 
            DriveConstants.kFrontLeftDriveEncoderReversed, 
            DriveConstants.kFrontLeftTurningEncoderReversed,
            frontLeftVisPt);
        frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
            DriveConstants.kFrontRightTurningMotorPort, 
            DriveConstants.kFrontRightAbsolutePort, 
            DriveConstants.kFrontRightKey, 
            DriveConstants.kFrontRightDriveEncoderReversed, 
            DriveConstants.kFrontRightTurningEncoderReversed,
            frontRightVisPt);
        backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
            DriveConstants.kBackLeftTurningMotorPort, 
            DriveConstants.kBackLeftAbsolutePort, 
            DriveConstants.kBackLeftKey, 
            DriveConstants.kBackLeftDriveEncoderReversed, 
            DriveConstants.kBackLeftTurningEncoderReversed,
            backLeftVisPt);
        backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
            DriveConstants.kBackRightTurningMotorPort, 
            DriveConstants.kBackRightAbsolutePort, 
            DriveConstants.kBackRightKey, 
            DriveConstants.kBackRightDriveEncoderReversed, 
            DriveConstants.kBackRightTurningEncoderReversed,
            backRightVisPt);

        //driveTrainSubsystem = new DriveTrainSubsystem(mod0,mod1,mod2,mod3);
        swerveSubsystem = new SwerveSubsystem(frontLeft, frontRight, backLeft, backRight, gyro);
        swerveSubsystem.setDefaultCommand(new JoystickToSwerve(swerveSubsystem, 
            () -> -driver.getRawAxis(1), 
            () -> -driver.getRawAxis(0), 
            () -> -driver.getRawAxis(4), 
            () -> driver.getRawButton(XboxController.Button.kRightBumper.value)
        ));
        
        zeroHeading = new JoystickButton(driver, XboxController.Button.kA.value).whenPressed(() -> swerveSubsystem.zeroHeading());

        resetEncoders = new JoystickButton(driver, XboxController.Button.kB.value).whenPressed(new InstantCommand(swerveSubsystem::resetEncoders,swerveSubsystem));
        vision = new VisionPhotonCamera("Global_Shutter_Camera", Units.inchesToMeters(16.75), 0, 0, 1, "Driver");
        drivetrainPoseEstimator = new DrivetrainPoseEstimator(this, vision);
    }

    public SwerveSubsystem getSwerveSubsystem(){
        return swerveSubsystem;
    }

    /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return gyro.getAngleZ();
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation3d(getGyroAngleX(), getGyroAngleY(), getGyroAngleZ()).toRotation2d();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }
}
