// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ModuleConstants {
    public static final double maxAbsEncoderVolts = 4.815;
    public static final double voltsToDegrees = (360/maxAbsEncoderVolts);
    public static final double voltsToRadians = (2*Math.PI/maxAbsEncoderVolts);
}
