package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class CameraSubsystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private AprilTagDetection currentTarget;

    // =====================
    // INIT
    // =====================
    public void init(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }

    // =====================
    // UPDATE (call every loop)
    // =====================
    public void update() {
        currentTarget = null;

        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return;

        for (AprilTagDetection tag : detections) {
            // Only care about scoring tags
            if (tag.id == 24 || tag.id == 20) {
                if (tag.ftcPose != null) {
                    currentTarget = tag;
                    return;
                }
            }
        }
    }

    // =====================
    // GETTERS
    // =====================
    public boolean hasTarget() {
        return currentTarget != null;
    }

    public double getBearing() {
        if (currentTarget == null) return 0.0;
        return currentTarget.ftcPose.bearing;
    }

    public int getTargetId() {
        if (currentTarget == null) return -1;
        return currentTarget.id;
    }

    public int getDesiredOrderFromTag() {
        if (currentTarget == null) return 1; // default safe order

        switch (currentTarget.id) {
            case 21: return 1;
            case 22: return 2;
            case 23: return 3;
            default: return 1;
        }
    }
}
