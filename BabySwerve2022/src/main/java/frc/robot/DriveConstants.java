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
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // top left
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // top right
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // bottom left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); // bottom right
     

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 5;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 6;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final String kFrontLeftDriveAbsoluteEncoderOffsetRadKey = "FrontLeftOffset"; 
    public static final String kFrontRightDriveAbsoluteEncoderOffsetRadKey = "FrontRightOffset";
    public static final String kBackLeftDriveAbsoluteEncoderOffsetRadKey = "BackLeftOffset"; 
    public static final String kBackRightDriveAbsoluteEncoderOffsetRadKey = "BackRightOffset"; 

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.9; 
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 6.27;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.13; 
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.87;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}
