package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import java.util.List;

@TeleOp(name = "Decode Mecanum + Color", group = "Drive")
public class asfasf extends OpMode {

    // =====================
// TURRET SERVOS
// =====================
    private Servo turret1, turret2;
    private double turretPos = 0.4;

    private static final double TURRET_HOME = 0.4;
    private static final double TURRET_RETURN_SPEED = 0.01;

    private double lastTurretTarget = 0.4;
    private static final double SERVO_SETTLED_EPSILON = 0.008;

    // =====================
// APRILTAG VISION
// =====================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    //All the states for intaking
    public enum IntakeState {
        BEFORE_START,
        RANDOM_START_TIMER,
        READY,
        INTAKE_DETECT,
        WAIT_FOR_READY,
        CHAMBERS_FULL,
        END
    }
    IntakeState intakeState = IntakeState.BEFORE_START;

    //All the servo values for the spinner that I can use
    final double SPIN_POS_1 = 0.98676;
    final double SPIN_POS_2 = 0.8467;
    final double SPIN_POS_3 = 0.69;
    final double SPIN_POS_4 = 0.552;

    final double SPIN_POS_MID_1 = 0.93;
    final double SPIN_POS_MID_2 = 0.77;
    final double SPIN_POS_MID_3 = 0.63;

    double[] POSITION_LIST = {
            SPIN_POS_1,
            SPIN_POS_MID_1,
            SPIN_POS_2,
            SPIN_POS_MID_2,
            SPIN_POS_3,
            SPIN_POS_MID_3,
            SPIN_POS_4
    };

    int NEXT_POSITION = 0;


    //SHOOTING STATES
    public enum ShootingState {
        READY,
        PUSHER_UP,
        SPINNER_MID,
        PUSHER_DOWN,
        SPINNER_FULL,
        NO_BALLS
    }
    ShootingState shootingState = ShootingState.NO_BALLS;
    int NEXT_SHOOT_POS = 0;
    //PUSHER POSITIONs
    final double PUSHER_POS_UP = 0.367;
    final double PUSHER_POS_DOWN = 0.9367;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Mechanisms
    private DcMotor intake;
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher;

    // Color sensor system
    private colorSensorDecode colorBench = new colorSensorDecode();

    //TIMERS
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime shootingTimer = new ElapsedTime();

    // Drive speed scaling
    private static final double SLOW_SPEED = 0.45; // default speed (45%)
    private static final double FAST_SPEED = 1.0;  // full speed

    @Override
    public void init() {
        intakeTimer.reset();
        shootingTimer.reset();
        // --------------------
        // HARDWARE MAP
        // --------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        // Init color sensors
        colorBench.init(hardwareMap);

        // --------------------
        // MOTOR DIRECTIONS
        // --------------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //shooter
        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

        // --------------------
        // TURRET SERVOS
        // --------------------
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret1.setPosition(turretPos);
        turret2.setPosition(turretPos);

        // --------------------
        // APRILTAG INIT
        // --------------------
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                        org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                        "Webcam"))
                .addProcessor(aprilTag)
                .build();


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
// --------------------
// MECANUM DRIVE (SCALED)
// --------------------
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

        // --------------------
        // INTAKE & SHOOTER
        // --------------------
        intake.setPower(gamepad1.right_trigger >= 0.4 ? 1 : 0);

        //pusher.setPosition(gamepad1.left_trigger >= 0.4 ? 0.45 : 0.9367);

        // Spinner positions
        if (gamepad1.a) spinner.setPosition(0.98676);
        if (gamepad1.b) spinner.setPosition(0.83676);
        if (gamepad1.x) spinner.setPosition(0.68676);
        if (gamepad1.y) spinner.setPosition(0.53676);
        if (gamepad1.dpad_down) spinner.setPosition(0.921);
        if (gamepad1.dpad_right) spinner.setPosition(0.771);
        if (gamepad1.dpad_left) spinner.setPosition(0.621);

        //SHOOTA
        if (gamepad1.left_bumper) {
            shoot1.setVelocity(1250);
            shoot2.setVelocity(1250);
        } else {
            shoot1.setVelocity(0);
            shoot2.setVelocity(0);
        }

        // --------------------
        // COLOR SENSORS
        // --------------------
        telemetry.addData("NEXT_POSITION pos: ", NEXT_POSITION);
        telemetry.addData("SHOOT_POSITION pos: ", NEXT_SHOOT_POS);
        telemetry.addData("INTAKE Current State: ", intakeState);
        telemetry.addData("SHOOTER Current State: ", shootingState);
        telemetry.addData("PUSHER POS: ", pusher.getPosition());
        telemetry.addData("SPINNER POS: ", spinner.getPosition());


        colorSensorDecode.DetectedColor c1 =
                colorBench.getDetectedColor(0, telemetry);
        colorSensorDecode.DetectedColor c2 =
                colorBench.getDetectedColor(1, telemetry);
        colorSensorDecode.DetectedColor c3 =
                colorBench.getDetectedColor(2, telemetry);
        colorSensorDecode.DetectedColor c4 =
                colorBench.getDetectedColor(3, telemetry);


        // --------------------
        // TELEMETRY
        // --------------------
        telemetry.addData("Drive FL", "%.2f", fl);
        telemetry.addData("Drive FR", "%.2f", fr);
        telemetry.addData("Drive BL", "%.2f", bl);
        telemetry.addData("Drive BR", "%.2f", br);

        telemetry.addLine("--- Color Sensors ---");
        telemetry.addData("Chamber 1", c1);
        telemetry.addData("Chamber 2", c2);
        telemetry.addData("Chamber 3", c3);
        telemetry.addData("Chamber 4", c4);


        telemetry.update();
        //Intake Finite State Code

        switch (intakeState) {
            case BEFORE_START:
                spinner.setPosition(POSITION_LIST[NEXT_POSITION]);
                intakeTimer.reset();
                intakeState = IntakeState.RANDOM_START_TIMER;
            case RANDOM_START_TIMER:
                if (intakeTimer.seconds() >= 2) {
                    intakeTimer.reset();
                    intakeState = IntakeState.READY;
                }
            case READY:
                spinner.setPosition(POSITION_LIST[NEXT_POSITION]);
                //Waiting for colorSensor1 input
                if (intakeTimer.seconds() >= 0.2 && (c4 == colorSensorDecode.DetectedColor.PURPLE || c4 == colorSensorDecode.DetectedColor.GREEN)) {
                    NEXT_POSITION += 2;
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_DETECT;
                }
                break;
            case INTAKE_DETECT:
                if (intakeTimer.seconds() >= 0.41) {
                    spinner.setPosition(POSITION_LIST[NEXT_POSITION]);
                    intakeTimer.reset();
                    intakeState = IntakeState.WAIT_FOR_READY;
                }
                break;
            case WAIT_FOR_READY:
                if (NEXT_POSITION >= 4) {
                    intakeTimer.reset();
                    intakeState = IntakeState.CHAMBERS_FULL;
                }

                if (intakeTimer.seconds() >= 0.3) {
                    intakeTimer.reset();
                    intakeState = IntakeState.READY;
                }
                break;
            case CHAMBERS_FULL:
                //((c4 == colorSensorDecode.DetectedColor.UNKNOWN) &&
                NEXT_POSITION = 0;

                if (intakeTimer.seconds() >= 0.67 && c2 == colorSensorDecode.DetectedColor.UNKNOWN && c3 == colorSensorDecode.DetectedColor.UNKNOWN && shootingState == ShootingState.NO_BALLS) {
                    intakeTimer.reset();
                    intakeState = IntakeState.BEFORE_START;
                }
                break;
        }

        //Shooter Finite State Code
        switch (shootingState) {
            case READY:
                pusher.setPosition(PUSHER_POS_DOWN);
                spinner.setPosition(POSITION_LIST[NEXT_SHOOT_POS]);
                // && ((c1 == colorSensorDecode.DetectedColor.PURPLE || c1 == colorSensorDecode.DetectedColor.GREEN) && (c2 == colorSensorDecode.DetectedColor.PURPLE || c2 == colorSensorDecode.DetectedColor.GREEN) && (c3 == colorSensorDecode.DetectedColor.PURPLE || c3 == colorSensorDecode.DetectedColor.GREEN))
                if ((gamepad1.right_bumper)) {
                    shootingTimer.reset();
                    shootingState = ShootingState.PUSHER_UP;
                }
                break;
            case PUSHER_UP:
                spinner.setPosition(POSITION_LIST[NEXT_SHOOT_POS]);
                if (shootingTimer.seconds() >= 0.5) {
                    pusher.setPosition(PUSHER_POS_UP);
                    if (pusher.getPosition() <= 0.51) {
                        NEXT_SHOOT_POS++;
                        shootingTimer.reset();
                        shootingState = ShootingState.SPINNER_MID;
                    }
                }
                break;
            case SPINNER_MID:
                if (shootingTimer.seconds() >= 0.5) {
                    spinner.setPosition(POSITION_LIST[NEXT_SHOOT_POS]);
                    if (spinner.getPosition() <= 0.93) {
                        NEXT_SHOOT_POS++;
                        shootingTimer.reset();
                        shootingState = ShootingState.PUSHER_DOWN;
                    }
                }
                break;
            case PUSHER_DOWN:
                if (shootingTimer.seconds() >= 0.5) {
                    pusher.setPosition(PUSHER_POS_DOWN);
                    if (pusher.getPosition() >= 0.93) {
                        shootingTimer.reset();
                        shootingState = ShootingState.SPINNER_FULL;
                    }
                }
                break;
            case SPINNER_FULL:
                if (shootingTimer.seconds() >= 1.2) {
                    shootingTimer.reset();
                    spinner.setPosition(POSITION_LIST[NEXT_SHOOT_POS]);
                    if (NEXT_SHOOT_POS >= 6) {
                        shootingState = ShootingState.NO_BALLS;
                    }else {
                        shootingState = ShootingState.PUSHER_UP;
                    }

                }
                break;
            case NO_BALLS:
                NEXT_SHOOT_POS = 0;
                shootingTimer.reset();
                pusher.setPosition(PUSHER_POS_DOWN);
                // || (c1 == colorSensorDecode.DetectedColor.PURPLE || c1 == colorSensorDecode.DetectedColor.GREEN)
                if ((((c4 == colorSensorDecode.DetectedColor.PURPLE || c4 == colorSensorDecode.DetectedColor.GREEN)) && (c2 == colorSensorDecode.DetectedColor.PURPLE || c2 == colorSensorDecode.DetectedColor.GREEN) && (c3 == colorSensorDecode.DetectedColor.PURPLE || c3 == colorSensorDecode.DetectedColor.GREEN)) && (intakeState == IntakeState.CHAMBERS_FULL)) {
                    shootingState = ShootingState.READY;
                }
        }

        //TURRET CODES
        if (gamepad1.left_trigger > 0.2) {
            // Manual override → go home
            returnTurretHome();
        } else {
            // Auto tracking
            updateTurretFromAprilTag();
        }
        telemetry.addData("Turret Mode",
                gamepad1.left_trigger > 0.2 ? "MANUAL RESET" : "AUTO TRACK");

        telemetry.addData("Turret Pos", turretPos);
        telemetry.update();
    }
    private void updateTurretFromAprilTag() {
            if (aprilTag == null || visionPortal == null) return;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.isEmpty()) return;

            AprilTagDetection tag = detections.get(0);
            if (tag.ftcPose == null) return;

            double bearing = tag.ftcPose.bearing;

            double kP = 1.0 / 6708;
            double target = turretPos + bearing * kP;

            target = Math.max(0.0, Math.min(1.0, target));

            // Smooth toward target
            turretPos += (target - turretPos) * 0.25;

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
