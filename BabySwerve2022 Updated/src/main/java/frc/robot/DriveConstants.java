// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(7.75);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(7.75);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right
     

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 5;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 6;

    public static final int kFrontLeftAbsolutePort = 0;
    public static final int kFrontRightAbsolutePort = 1;
    public static final int kBackLeftAbsolutePort = 2;
    public static final int kBackRightAbsolutePort = 3;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftAbsoluteReversed = true;
    public static final boolean kFrontRightAbsoluteReversed = true;
    public static final boolean kBackLeftAbsoluteReversed = true;
    public static final boolean kBackRightAbsoluteReversed = true;

    public static final String kFrontLeftKey = "FrontLeft"; 
    public static final String kFrontRightKey = "FrontRight";
    public static final String kBackLeftKey = "BackLeft"; 
    public static final String kBackRightKey = "BackRight"; 

    public static final double kFrontLeftAbsoluteOffsetRad = 2.884; 
    public static final double kFrontRightAbsoluteOffsetRad = 0;
    public static final double kBackLeftAbsoluteOffsetRad = 5.224; 
    public static final double kBackRightAbsoluteOffsetRad = 6.074;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 0.05;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = .08;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = .4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI;
}
