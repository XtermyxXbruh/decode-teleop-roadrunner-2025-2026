package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.colorSensorDecode;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@TeleOp(name = "NewClassTest", group = "Drive")
public class chatty extends OpMode {

    // =====================
    // DRIVE
    // =====================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private static final double SLOW_SPEED = 0.45;
    private static final double FAST_SPEED = 1.0;

    // =====================
    // MECHANISMS
    // =====================
    private DcMotor intake;
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher;

    double SHOOTER_TICK_SPEED = 1200;

    // =====================
    // SPINNER POSITIONS
    // =====================
    final double[] POSITION_LIST = {
            0.985, 0.94,
            0.853,  0.79,
            0.6877,  0.64,
            0.5567,0.49,
            0.41,  0.34,
            0.2467,0.17,
            0.1,   0.2,
            0
    };

    // =====================
    // SUBSYSTEMS
    // =====================
    private IntakeSubsystem intakeSubsystem;
    private colorSensorDecode colorBench;

    private colorSensorDecode.DetectedColor c2, c3, c4;
    private Shooter shooter;

    // =====================
    // TURRET
    // =====================
    private Servo turret1, turret2;
    private double turretPos = 0.38;

    private static final double TURRET_HOME = 0.38;
    private static final double TURRET_RETURN_SPEED = 0.01;

    // =====================
    // APRILTAG VISION
    // =====================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    //GREEN BALL SHOOT ORDER
    int BALL_DESIRED_ORDER = 1;
    int GREEN_BALL_POS = 1;

    @Override
    public void init() {

        // Drive
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

        // Mechanisms
        intake = hardwareMap.get(DcMotor.class, "intake");

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");

        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        // Turret servos
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);

    // AprilTag
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();


        // Color sensors
        colorBench = new colorSensorDecode();
        colorBench.init(hardwareMap);

    // Intake subsystem
        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.init(hardwareMap);

        shooter = new Shooter(
                shoot1,
                shoot2,
                spinner,
                pusher,
                POSITION_LIST
        );

        shooter.setStartShootPos(0);

        shooter.setStartShootPos(0);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --------------------
        // DRIVE
        // --------------------
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double speed = (gamepad1.left_trigger > 0.2)
                ? FAST_SPEED
                : SLOW_SPEED;

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

        boolean overrideShoot = gamepad2.y;

        //TODO: ALL THE BUUUUTOOOOONS
        if (overrideShoot) {
            intakeSubsystem.forceChambersFull();
            shooter.forceReady();
        }

        if (gamepad2.dpad_up) {
            SHOOTER_TICK_SPEED = 1767;
        }
        if (gamepad2.dpad_down) {
            SHOOTER_TICK_SPEED = 967;
        }
        if (gamepad2.dpad_right || gamepad2.dpad_left) {
            SHOOTER_TICK_SPEED = 1200;
        }

        // --------------------
        // INTAKE
        // --------------------
        if (gamepad1.right_trigger > 0.2) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

// --------------------
// COLOR SENSORS
// --------------------
        c2 = colorBench.getDetectedColor(1, telemetry);
        c3 = colorBench.getDetectedColor(2, telemetry);
        c4 = colorBench.getDetectedColor(3, telemetry);

// ---------
// GREEN BALL DETECTION SORTING
// ---------
        if (c4 == colorSensorDecode.DetectedColor.GREEN) GREEN_BALL_POS = 1;
        else if (c2 == colorSensorDecode.DetectedColor.GREEN) GREEN_BALL_POS = 2;
        else if (c3 == colorSensorDecode.DetectedColor.GREEN) GREEN_BALL_POS = 3;

        if (gamepad2.dpad_up) BALL_DESIRED_ORDER = 1;
        else if (gamepad2.dpad_right) BALL_DESIRED_ORDER = 2;
        else if (gamepad2.dpad_left) BALL_DESIRED_ORDER = 3;

        if (shooter.isIdle()) {
            shooter.setStartIndexFromOrder(
                    BALL_DESIRED_ORDER,
                    GREEN_BALL_POS
            );
        }
// --------------------
// INTAKE FSM
// --------------------
        intakeSubsystem.update(
                c2,
                c3,
                c4,
                shooter.getState()
        );

// --------------------
// SPINNER OWNERSHIP LOGIC
// --------------------
        if (!intakeSubsystem.chambersFull()) {
            // Intake FSM owns spinner
            int spinnerIndex = intakeSubsystem.getNextSpinnerIndex();
            spinnerIndex = Math.max(0, Math.min(spinnerIndex, POSITION_LIST.length - 1));
            spinner.setPosition(POSITION_LIST[spinnerIndex]);
        }
// else:
// Shooter FSM controls spinner internally


// --------------------
// SHOOTER FSM
// --------------------
        boolean shootCommand = gamepad2.right_bumper;
        boolean intakeFull = intakeSubsystem.chambersFull();

        shooter.update(shootCommand, intakeFull);


        if (!shooter.isIdle()) {
            shoot1.setVelocity(SHOOTER_TICK_SPEED);
            shoot2.setVelocity(SHOOTER_TICK_SPEED);
        } else {
            shoot1.setVelocity(0);
            shoot2.setVelocity(0);
        }

        // --------------------
// TURRET CONTROL
// --------------------
        if (gamepad2.left_trigger > 0.2) {
            // Manual override → return home
            returnTurretHome();
        } else {
            // Auto track AprilTag
            updateTurretFromAprilTag();
        }

        telemetry.addData("Turret Pos", turretPos);


        telemetry.addData("Intake State", intakeSubsystem.getState());
        telemetry.addData("Next Spinner Index", intakeSubsystem.getNextSpinnerIndex());
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.update();
    }
    private void updateTurretFromAprilTag() {
        if (aprilTag == null || visionPortal == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return;

        AprilTagDetection target = null;

        // ONLY TRACK ID 24 OR 20
        for (AprilTagDetection tag : detections) {
            if (tag.id == 24 || tag.id == 20) {
                target = tag;
                break;
            }
        }

        if (target == null || target.ftcPose == null) return;

        double bearing = target.ftcPose.bearing;

        // Proportional control
        double kP = 1.0 / 410;
        double desired = turretPos + bearing * kP;

        desired = Math.max(0.0, Math.min(1.0, desired));

        // Smooth movement
        turretPos += (desired - turretPos) * 0.25;

        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);
    }

    private void returnTurretHome() {
        double error = TURRET_HOME - turretPos;

        if (Math.abs(error) < 0.002) {
            turretPos = TURRET_HOME;
        } else {
            turretPos += Math.signum(error) * TURRET_RETURN_SPEED;
        }

        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);
    }

}