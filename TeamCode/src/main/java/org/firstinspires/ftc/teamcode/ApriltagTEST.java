package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Servo Template", group = "Concept")
public class ApriltagTEST extends LinearOpMode {

    /* =========================
       Motors
       ========================= */
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    /* =========================
       Servos (ADD YOUR SERVOS HERE)
       ========================= */
    private Servo turret1, turret2;
    double servoPos = 0.415;
    double remainingError = 0;
    double FAST_SPEED = 1;
    double SLOW_SPEED = 0.5;
    // ← example servo

    /* =========================
       Vision
       ========================= */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    double servoPosition = 0.5;
    @Override
    public void runOpMode() {

        /* =========================
           Hardware Init
           ========================= */
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setPosition(0.415);
        turret2.setPosition(0.415);

        initAprilTag();

        telemetry.addLine("AprilTag Servo Template Ready");

        telemetry.update();

        waitForStart();

        /* =========================
           MAIN LOOP
           ========================= */
        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

// Speed modifier
            double speedMultiplier =
                    (gamepad1.left_trigger > 0.2) ? FAST_SPEED : SLOW_SPEED;

            double fl = (y + x + rx) * speedMultiplier;
            double bl = (y - x + rx) * speedMultiplier;
            double fr = (y - x - rx) * speedMultiplier;
            double br = (y + x - rx) * speedMultiplier;

// Normalize so nothing exceeds |1|
            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl),
                            Math.max(Math.abs(fr), Math.abs(br))));

            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);


            AprilTagDetection tag = getAprilTag();

            if (tag != null && tag.ftcPose != null) {
                controlFromAprilTag(tag);
            } else {
            }

            telemetry.update();
            sleep(20);
        }
    }

    /* =========================
       READ APRILTAG DATA
       ========================= */
    private AprilTagDetection getAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;
        return detections.get(0);
    }

    /* =========================================================
       ========== ADD SERVO / MOTOR CODE HERE ==================
       ========================================================= */
    private void controlFromAprilTag(AprilTagDetection tag) {

        double range   = tag.ftcPose.range;    // inches
        double bearing = tag.ftcPose.bearing;  // degrees
        int tagId      = tag.id;


        // =======================
        // ADD SERVO CODE HERE
        // =======================
        remainingError = Math.abs(turret1.getPosition() - servoPos);

        if ((remainingError <= 0.01) && (tagId == 24)) {
            servoPos += (bearing/1234.5);
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            turret1.setPosition(servoPos);
            turret2.setPosition(servoPos);
        }


        telemetry.addData("target _POS ", servoPos);
        telemetry.addData("bearing", bearing);
        telemetry.addData("remainingerror", remainingError);
        telemetry.addData("ser VO POS", turret1.getPosition());
        telemetry.update();


        // =======================
        // ADD MOTOR CODE HERE
        // =======================

        if (tagId == 4) {

        } else {

        }
    }

    /* =========================
       APRILTAG INIT
       ========================= */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }
}
