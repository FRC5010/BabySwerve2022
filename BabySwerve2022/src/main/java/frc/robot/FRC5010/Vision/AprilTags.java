// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class AprilTags {
    public static final ArrayList<AprilTag> aprilTagPoses = new ArrayList<>();

    public static enum AprilTag {
        // TODO: Define April Tag positions here.
        ID0("Blue Hangar Panel", Units.inchesToMeters(-0.139), Units.inchesToMeters(298.383), Units.inchesToMeters(34.876), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID1("Blue Hangar Truss - Hub", Units.inchesToMeters(127.272), Units.inchesToMeters(216.01), Units.inchesToMeters(67.932), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID2("Blue Hangar Truss - Side", Units.inchesToMeters(120.78), Units.inchesToMeters(209.863), Units.inchesToMeters(54.182), Units.degreesToRadians(-90), Units.degreesToRadians(0)),
        ID3("Blue Station 2 Wall", Units.inchesToMeters(0.157), Units.inchesToMeters(199.155), Units.inchesToMeters(31.75), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID4("Blue Station 3 Wall", Units.inchesToMeters(0.157), Units.inchesToMeters(138.287), Units.inchesToMeters(31.75), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID5("Blue Terminal Near Station", Units.inchesToMeters(4.768), Units.inchesToMeters(67.631), Units.inchesToMeters(35.063), Units.degreesToRadians(46.25), Units.degreesToRadians(0)),
        ID6("Blue Mid Terminal", Units.inchesToMeters(34.382), Units.inchesToMeters(37.059), Units.inchesToMeters(35.063), Units.degreesToRadians(46.25), Units.degreesToRadians(0)),
        ID7("Blue End Terminal", Units.inchesToMeters(63.586), Units.inchesToMeters(6.191), Units.inchesToMeters(35.063), Units.degreesToRadians(46.25), Units.degreesToRadians(0)),
        ID10("Red Hangar Panel", Units.inchesToMeters(648.139), Units.inchesToMeters(25.617), Units.inchesToMeters(34.876), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID11("Red Hangar Truss - Hub", Units.inchesToMeters(521.063), Units.inchesToMeters(108.01), Units.inchesToMeters(67.932), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID12("Red Hangar Truss - Side", Units.inchesToMeters(527.22), Units.inchesToMeters(114.167), Units.inchesToMeters(54.182), Units.degreesToRadians(90), Units.degreesToRadians(0)),
        ID13("Red Station 2 Wall", Units.inchesToMeters(647.843), Units.inchesToMeters(125.02), Units.inchesToMeters(31.75), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID14("Red Station 3 Wall", Units.inchesToMeters(647.843), Units.inchesToMeters(185.714), Units.inchesToMeters(31.75), Units.degreesToRadians(0), Units.degreesToRadians(0)),
        ID15("Red Terminal Near Station", Units.inchesToMeters(643.111), Units.inchesToMeters(256.495), Units.inchesToMeters(35.188), Units.degreesToRadians(223.8), Units.degreesToRadians(0)),
        ID16("Red Mid Terminal", Units.inchesToMeters(613.799), Units.inchesToMeters(287.114), Units.inchesToMeters(35.063), Units.degreesToRadians(223.8), Units.degreesToRadians(0)),
        ID17("Red End Terminal", Units.inchesToMeters(584.535), Units.inchesToMeters(317.682), Units.inchesToMeters(35.063), Units.degreesToRadians(223.8), Units.degreesToRadians(0)),
        ID18(0, 0, 0, 0, 0),
        ID19(0, 0, 0, 0, 0),
        ID20(0, 0, 0, 0, 0),
        ID21(0, 0, 0, 0, 0),
        ID22(0, 0, 0, 0, 0),
        ID23(0, 0, 0, 0, 0),
        ID24(0, 0, 0, 0, 0),
        ID25(0, 0, 0, 0, 0),
        ID26(0, 0, 0, 0, 0),
        ID27(0, 0, 0, 0, 0),
        ID28(0, 0, 0, 0, 0),
        ID29(0, 0, 0, 0, 0),
        ID30(0, 0, 0, 0, 0);

        // --------------------------------------
        // The rest of this is just boiler-plate for setting up the aprilTagPoses array
        // with all of the defined april tags.
        public Pose3d pose;
        public String fieldDescriptor;

        private AprilTag(double xPos, double yPos, double zPos, double pitch, double yaw) {
            this.fieldDescriptor = this.name();
            pose = new Pose3d(new Translation3d(xPos, yPos, zPos),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(yaw)));
        }

        private AprilTag(String fieldDescriptor, double xPos, double yPos, double zPos, double pitch, double yaw) {
            this.fieldDescriptor = fieldDescriptor;
            pose = new Pose3d(new Translation3d(xPos, yPos, zPos),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(yaw)));
        }
    }

    static {
        for (AprilTag aprilTag : AprilTag.values()) {
            aprilTagPoses.add(aprilTag);
        }
    }
}
