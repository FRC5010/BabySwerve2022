// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DriveConstants;
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
    AHRS gyro;
    private Button zeroHeading;
    private Button resetEncoders;
    private final Mechanism2d mech2dSim = new Mechanism2d(60, 60);
    private final MechanismRoot2d frontLeftSimPt = mech2dSim.getRoot(DriveConstants.kFrontLeftKey, 40, 20);
    private final MechanismRoot2d frontRightSimPt = mech2dSim.getRoot(DriveConstants.kFrontLeftKey, 40, 40);
    private final MechanismRoot2d backLeftSimPt = mech2dSim.getRoot(DriveConstants.kFrontLeftKey, 20, 20);
    private final MechanismRoot2d backRightSimPt = mech2dSim.getRoot(DriveConstants.kFrontLeftKey, 20, 40);
            
    public Drive(AHRS gyro){

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

        this.gyro = gyro;
        Joystick driver = new Joystick(0);
        frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, 
            DriveConstants.kFrontLeftTurningMotorPort, 
            DriveConstants.kFrontLeftAbsolutePort, 
            DriveConstants.kFrontLeftKey, 
            DriveConstants.kFrontLeftDriveEncoderReversed, 
            DriveConstants.kFrontLeftTurningEncoderReversed,
            frontLeftSimPt);
        frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, 
            DriveConstants.kFrontRightTurningMotorPort, 
            DriveConstants.kFrontRightAbsolutePort, 
            DriveConstants.kFrontRightKey, 
            DriveConstants.kFrontRightDriveEncoderReversed, 
            DriveConstants.kFrontRightTurningEncoderReversed,
            frontRightSimPt);
        backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, 
            DriveConstants.kBackLeftTurningMotorPort, 
            DriveConstants.kBackLeftAbsolutePort, 
            DriveConstants.kBackLeftKey, 
            DriveConstants.kBackLeftDriveEncoderReversed, 
            DriveConstants.kBackLeftTurningEncoderReversed,
            backLeftSimPt);
        backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, 
            DriveConstants.kBackRightTurningMotorPort, 
            DriveConstants.kBackRightAbsolutePort, 
            DriveConstants.kBackRightKey, 
            DriveConstants.kBackRightDriveEncoderReversed, 
            DriveConstants.kBackRightTurningEncoderReversed,
            backRightSimPt);

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
    }
}
