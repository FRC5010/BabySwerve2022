// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Driving;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class Drive {
    SwerveModule mod0;
    SwerveModule mod1;
    SwerveModule mod2;
    SwerveModule mod3;
    private DriveTrainSubsystem driveTrainSubsystem;
    private Driving driving;
    
    public Drive(){
        Joystick driver = new Joystick(0);

        CANSparkMax speedMotor0 = new CANSparkMax(1, MotorType.kBrushless);
        CANSparkMax angleMotor0 = new CANSparkMax(2,MotorType.kBrushless);
        mod0 = new SwerveModule(speedMotor0, angleMotor0, 0, 0, false);

        CANSparkMax speedMotor1 = new CANSparkMax(3, MotorType.kBrushless);
        CANSparkMax angleMotor1 = new CANSparkMax(4,MotorType.kBrushless);
        mod1 = new SwerveModule(speedMotor1, angleMotor1, 1, 0, false);

        CANSparkMax speedMotor2 = new CANSparkMax(5, MotorType.kBrushless);
        CANSparkMax angleMotor2 = new CANSparkMax(6,MotorType.kBrushless);
        mod2 = new SwerveModule(speedMotor2, angleMotor2, 2, 0, false);

        CANSparkMax speedMotor3 = new CANSparkMax(7, MotorType.kBrushless);
        CANSparkMax angleMotor3 = new CANSparkMax(8,MotorType.kBrushless);
        mod3 = new SwerveModule(speedMotor3, angleMotor3, 3, 0, false);

        driveTrainSubsystem = new DriveTrainSubsystem(mod0,mod1,mod2,mod3);

        driving = new Driving(driveTrainSubsystem, driver);
        driveTrainSubsystem.setDefaultCommand(driving);

    }
}
