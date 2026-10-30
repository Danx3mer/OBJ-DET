package com.stuypulse.robot.constants;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;

public class Cameras {
    public static Camera[] LimelightCameras = new Camera[] {
        new Camera("Live!_Cam_Sync_1080p", new Transform3d(
            0, 0, 0, 
            new Rotation3d(0, 0, 0)
        ))
    };
    public static record Camera(String name, Transform3d location) {};
}
