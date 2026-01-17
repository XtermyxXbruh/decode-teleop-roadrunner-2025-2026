package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class CameraSubsystem {

    public enum Mode {
        AUTO,
        TELEOP
    }

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private AprilTagDetection aimTarget;    // 20 / 24
    private AprilTagDetection orderTarget;  // 21 / 22 / 23

    private Mode mode = Mode.TELEOP;

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
    // MODE SET
    // =====================
    public void setMode(Mode mode) {
        this.mode = mode;
    }

    // =====================
    // UPDATE (call every loop)
    // =====================
    public void update() {
        aimTarget = null;
        orderTarget = null;

        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return;

        for (AprilTagDetection tag : detections) {
            if (tag.ftcPose == null) continue;

            // Aiming tags (always allowed)
            if (tag.id == 20 || tag.id == 24) {
                aimTarget = tag;
            }

            // Desired order tags (AUTO ONLY)
            if (mode == Mode.AUTO && tag.id >= 21 && tag.id <= 23) {
                orderTarget = tag;
            }
        }
    }

    // =====================
    // AIMING GETTERS
    // =====================
    public boolean hasAimTarget() {
        return aimTarget != null;
    }

    public double getAimBearing() {
        if (aimTarget == null) return 0.0;
        return aimTarget.ftcPose.bearing;
    }

    public int getAimTargetId() {
        if (aimTarget == null) return -1;
        return aimTarget.id;
    }

    // =====================
    // AUTO ONLY — ORDER
    // =====================
    public boolean hasOrderTarget() {
        return orderTarget != null;
    }

    public int getDesiredOrderFromTag() {
        if (orderTarget == null) return 1; // safe default

        switch (orderTarget.id) {
            case 21: return 1;
            case 22: return 2;
            case 23: return 3;
            default: return 1;
        }
    }
}
