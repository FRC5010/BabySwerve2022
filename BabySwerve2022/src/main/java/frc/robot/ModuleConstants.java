// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ModuleConstants {

    // neo 550 sysid values
    public static final double kSC = 0.55641;
    public static final double kVC = 0.064889;
    public static final double kAC = 0.0025381;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    // pid values for the neo 550
    public static final double kPTurning = 0.052037 * 1.5;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;

    // physical values
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kDriveMotorGearRatio = 1/5.25;
    public static final double kTurningMotorGearRatio = 1/((5.33) * 10.5); // not 12:1 but 10:5 for gearbox, ultraplanetaries are not nominal
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // conversions
    public static final double maxAbsEncoderVolts = 4.815;
    public static final double voltsToDegrees = (360/maxAbsEncoderVolts);
    public static final double voltsToRadians = (2*Math.PI/maxAbsEncoderVolts);
}
