// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DriveConstants;
import frc.robot.commands.JoystickToSwerve;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Drive {
    SwerveModule backRight;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule frontLeft;
    private SwerveSubsystem swerveSubsystem;
    AHRS gyro;
    private Button zeroHeading;
    private Button resetEncoders;
    
    public Drive(AHRS gyro){

        if(!Preferences.containsKey(DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRadKey)){
            Preferences.setDouble(DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRadKey, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRadKey)){
            Preferences.setDouble(DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRadKey, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRadKey)){
            Preferences.setDouble(DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRadKey, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);    
        }
        if(!Preferences.containsKey(DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRadKey)){
            Preferences.setDouble(DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRadKey, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);    
        }


        this.gyro = gyro;
        Joystick driver = new Joystick(0);
        frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurningMotorPort, 
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRadKey, 
            DriveConstants.kFrontLeftDriveEncoderReversed, 
            DriveConstants.kFrontLeftTurningEncoderReversed);
        frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurningMotorPort, 
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRadKey, 
            DriveConstants.kFrontRightDriveEncoderReversed, 
            DriveConstants.kFrontRightTurningEncoderReversed);
        backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftTurningMotorPort, 
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRadKey, 
            DriveConstants.kBackLeftDriveEncoderReversed, 
            DriveConstants.kBackLeftTurningEncoderReversed);
        backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightTurningMotorPort, 
            DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRadKey, 
            DriveConstants.kBackRightDriveEncoderReversed, 
            DriveConstants.kBackRightTurningEncoderReversed);

        //driveTrainSubsystem = new DriveTrainSubsystem(mod0,mod1,mod2,mod3);
        swerveSubsystem = new SwerveSubsystem(frontLeft, frontRight, backLeft, backRight, gyro);
        swerveSubsystem.setDefaultCommand(new JoystickToSwerve(swerveSubsystem, 
            () -> -driver.getRawAxis(1), 
            () -> driver.getRawAxis(0), 
            () -> -driver.getRawAxis(4), 
            () -> !driver.getRawButton(XboxController.Button.kX.ordinal())
        ));
        
        zeroHeading = new JoystickButton(driver, XboxController.Button.kA.ordinal()).whenPressed(() -> swerveSubsystem.zeroHeading());

        resetEncoders = new JoystickButton(driver, XboxController.Button.kB.ordinal()).whenPressed(new InstantCommand(swerveSubsystem::resetEncoders,swerveSubsystem));
    }
}
