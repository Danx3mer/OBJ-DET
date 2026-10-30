package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Settings;


public class PhotonVision extends Mechanism {
    private static final PhotonVision instance;
   
    static {
        instance = new PhotonVision(AprilTagFields.k2026RebuiltAndymark);
    }
   
    public static PhotonVision getInstance() {
        return instance;
    }

    private final VisionSystemSim visionSim;

    private final PhotonCamera[] realCameras = new PhotonCamera[Cameras.LimelightCameras.length];
    private final PhotonCameraSim[] cameraSims = new PhotonCameraSim[Cameras.LimelightCameras.length];
    private final PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[Cameras.LimelightCameras.length];
    private final Transform3d[] cameraTransforms = new Transform3d[Cameras.LimelightCameras.length];

    private final double FUEL_RADIUS = Units.inchesToMeters(2.955);
    private final Pose3d DEFAULT_POSE = new Pose3d();

    private final StructArrayPublisher<Pose3d> fuel;

    private PhotonVision(AprilTagFields fieldLayout) {
        this.visionSim = new VisionSystemSim("main");
       
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(fieldLayout);
        visionSim.addAprilTags(layout);

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            Camera camera = Cameras.LimelightCameras[i];

            PhotonCamera realCamera = new PhotonCamera(camera.name());
            realCamera.setPipelineIndex(0);
            Transform3d cameraTransform = new Transform3d(camera.location().getTranslation(), camera.location().getRotation());
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout, cameraTransform);

            realCameras[i] = realCamera;
            poseEstimators[i] = poseEstimator;
            cameraTransforms[i] = cameraTransform;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProps = new SimCameraProperties();

                PhotonCameraSim cameraSim = new PhotonCameraSim(realCamera, cameraProps);
                cameraSims[i] = cameraSim;
                visionSim.addCamera(cameraSim, cameraTransform);
            }
        }
        final NetworkTableInstance table = NetworkTableInstance.getDefault();
        this.fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    private Optional<Pose3d> getFuelFieldPose(
        Pose3d currentRobotPose, Transform3d robotToCamera, PhotonTrackedTarget target) {

        final double yawRad = Units.degreesToRadians(-target.getYaw()); // negative
        final double pitchRad = Units.degreesToRadians(target.getPitch());

        SmartDashboard.putNumber("Vision/Fuel Yaw", Units.radiansToDegrees(yawRad));
        SmartDashboard.putNumber("Vision/Fuel Pitch", Units.radiansToDegrees(pitchRad));

        final double cameraPitchRad = -robotToCamera.getRotation().getY(); // negative

        final double distance = PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            FUEL_RADIUS,
            cameraPitchRad,
            pitchRad
        );

        SmartDashboard.putNumber("Vision/target_" + target.getDetectedObjectClassID() + "distance_m", distance);
        if (distance <= 0 || Double.isNaN(distance)) {
            return Optional.empty();
        }

        final Translation3d camToPiece = new Translation3d(PhotonUtils.estimateCameraToTargetTranslation(distance, new Rotation2d(yawRad)));

        final Pose3d fieldToCamera = currentRobotPose.transformBy(robotToCamera);
        final Pose3d fieldToPiece = fieldToCamera.transformBy(
            new Transform3d(camToPiece, new Rotation3d())
        );

        return Optional.of(fieldToPiece);
    }

    private volatile List<Pose3d> fuelPoses = new ArrayList<>();

    public void periodic() {
        if (!Settings.EnabledSubsystems.VISION) {
            return;
        }

        boolean foundNewResult = false;

        if (Robot.isSimulation()) {
            visionSim.update(DEFAULT_POSE);
        }

        for (int i = 0; i < realCameras.length; i++) {
            final PhotonCamera currentCamera = realCameras[i];

            final List<PhotonPipelineResult> results = currentCamera.getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                if (!result.hasTargets()) {
                    continue;
                }

                if(result.getBestTarget() == null) continue;

                SmartDashboard.putNumber("POSE/timestamp", result.getTimestampSeconds());

                for(PhotonTrackedTarget target : result.targets) {
                    if(target.getDetectedObjectClassID() != Settings.Vision.FUEL_CLASS_ID) {
                        continue;
                    }

                    if(!foundNewResult) {
                        fuelPoses.clear();
                        foundNewResult = true;
                    }

                    var fuelPose = this.getFuelFieldPose(DEFAULT_POSE, cameraTransforms[i], target);
                    if (fuelPose.isPresent()) {
                        fuelPoses.add(fuelPose.get());
                    }
                }
            }
        }
        fuel.accept(fuelPoses.toArray(Pose3d[]::new));
    }
}