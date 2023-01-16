// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final double CAMERA_CAL_DISTANCE = 120;
    public static String SBTabVisionDisplay ="Vision";
    public static final double cameraHeight = Units.inchesToMeters(3);
    public static final double cameraAngle = Units.degreesToRadians(-20);
    public static final Transform3d kCameraToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(7.125), 0.0, Units.inchesToMeters(17)), new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
}
