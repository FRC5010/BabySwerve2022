// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kPXController = 0.55641/12;
    public static final double kPYController = 0.55641/12;
    public static final double kThetaController = 0.052037 * 4;

    public static final TrapezoidProfile.Constraints kThetaControllerConstrants = 
        new TrapezoidProfile.Constraints(
            Math.PI,
            Math.PI
        );

}
