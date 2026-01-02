package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "FULL TELEOP + TURRET", group = "Drive")
public class FINAL_TELEOP extends OpMode {

    /* =========================
       DRIVE
       ========================= */
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private static final double SLOW_SPEED = 0.45;
    private static final double FAST_SPEED = 1.0;

    /* =========================
       MECHANISMS
       ========================= */
    private DcMotor intake, shoot1, shoot2;
    private Servo spinner, pusher;

    /* =========================
       TURRET
       ========================= */
    private Servo turret1, turret2;
    private double turretPos = 0.415;
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;
    private static final double TURRET_KP  = 1.0 / 1234.5; // your scaling
    private static final double TURRET_DEADBAND = 0.4;    // degrees

    /* =========================
       VISION
       ========================= */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /* =========================
       COLOR SYSTEM
       ========================= */
    private colorSensorDecode colorBench = new colorSensorDecode();

    /* =========================
       TIMERS
       ========================= */
    private ElapsedTime turretTimer = new ElapsedTime();

    @Override
    public void init() {

        /* -------- Drive -------- */
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* -------- Mechanisms -------- */
        intake = hardwareMap.get(DcMotor.class, "intake");
        shoot1 = hardwareMap.get(DcMotor.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotor.class, "shoot2");

        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        /* -------- Turret -------- */
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);

        /* -------- Color -------- */
        colorBench.init(hardwareMap);

        /* -------- Vision -------- */
        initAprilTag();

        telemetry.addLine("FULL TELEOP READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* =========================
           DRIVETRAIN
           ========================= */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double speed =
                (gamepad1.left_trigger > 0.2) ? FAST_SPEED : SLOW_SPEED;

        double fl = (y + x + rx) * speed;
        double bl = (y - x + rx) * speed;
        double fr = (y - x - rx) * speed;
        double br = (y + x - rx) * speed;

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

        /* =========================
           MECHANISMS
           ========================= */
        intake.setPower(gamepad1.right_trigger >= 0.4 ? 1 : 0);

        if (gamepad1.left_bumper) {
            shoot1.setPower(1);
            shoot2.setPower(1);
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }

        //syot



        /* =========================
           APRILTAG TURRET
           ========================= */
        AprilTagDetection tag = getAprilTag();

        if (tag != null && tag.ftcPose != null && tag.id == 24) {

            double bearing = tag.ftcPose.bearing;

            if (Math.abs(bearing) > TURRET_DEADBAND) {
                turretPos += bearing * TURRET_KP;
                turretPos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretPos));

                turret1.setPosition(turretPos);
                turret2.setPosition(turretPos);
            }

            telemetry.addData("TAG", tag.id);
            telemetry.addData("Bearing", bearing);
        }

        telemetry.addData("Turret Pos", turretPos);
        telemetry.update();
    }

    /* =========================
       APRILTAG HELPERS
       ========================= */
    private AprilTagDetection getAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;
        return detections.get(0);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }
}
