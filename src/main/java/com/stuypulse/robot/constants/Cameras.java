package com.stuypulse.robot.constants;

import org.wpilib.math.geometry.Transform3d;

public class Cameras {
    public static Camera[] LimelightCameras = new Camera[] {
        new Camera("openaicam", new Transform3d())
    };
    public static record Camera(String name, Transform3d location) {};
}
