// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DriveConstants;
import frc.robot.commands.Driving;
import frc.robot.commands.JoystickToSwerve;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Drive {
    SwerveModule bottomRight;
    SwerveModule topRight;
    SwerveModule bottomLeft;
    SwerveModule topLeft;
    private DriveTrainSubsystem driveTrainSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private Driving driving;
    AHRS gyro;
    
    public Drive(AHRS gyro){
        this.gyro = gyro;
        Joystick driver = new Joystick(0);
        topRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, false);

        topLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, false);

        bottomLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, true);

        bottomRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, false);

        //driveTrainSubsystem = new DriveTrainSubsystem(mod0,mod1,mod2,mod3);
        swerveSubsystem = new SwerveSubsystem(bottomRight, topRight, bottomLeft, topLeft, gyro);
        swerveSubsystem.setDefaultCommand(new JoystickToSwerve(swerveSubsystem, 
            () -> driver.getRawAxis(0), 
            () -> -driver.getRawAxis(1), 
            () -> driver.getRawAxis(4), 
            () -> !driver.getRawButton(1)
        ));
        
        new JoystickButton(driver, 2).whenPressed(() -> swerveSubsystem.zeroHeading());


        //driving = new Driving(driveTrainSubsystem, driver);
        //driveTrainSubsystem.setDefaultCommand(driving);

    }
}
