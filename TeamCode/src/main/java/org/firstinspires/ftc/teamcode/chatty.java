package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.colorSensorDecode;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import com.qualcomm.robotcore.util.ElapsedTime;


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
            0.878,
            0.727,
            0.5855,
            0.433,
            0.26,
            0.13,
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

    private static double TURRET_HOME = 0.38;
    private static final double TURRET_RETURN_SPEED = 0.01;

    //GREEN BALL SHOOT ORDER
    int BALL_DESIRED_ORDER = 1;
    int GREEN_BALL_POS = 1;

    double lastServoPos;

    //camera
    private CameraSubsystem camera;

    private double PUSHER_UP = 0.567;
    private double PUSHER_DOWN = 0.27167;

    ElapsedTime shootingAccelTimer = new ElapsedTime();
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

        //camera
        camera = new CameraSubsystem();
        camera.init(hardwareMap);
        camera.setMode(CameraSubsystem.Mode.TELEOP);

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

        pusher.setPosition(0.05);

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

        if (spinner.getPosition() == POSITION_LIST[0]
                || spinner.getPosition() == POSITION_LIST[2]
                || spinner.getPosition() == POSITION_LIST[4]) {
            lastServoPos = spinner.getPosition();
        }

        // --------------------
        // TURRET CONTROL
        // --------------------
        camera.update();

        if (gamepad2.left_trigger > 0.2) {
            returnTurretHome();
        } else if (camera.hasAimTarget()) {
            double bearing = camera.getAimBearing();

            double kP = 1.0 / 270;
            double desired = turretPos + bearing * kP;
            desired = Math.max(0.0, Math.min(1.0, desired));
            turretPos += (desired - turretPos) * 0.25;

            turret1.setPosition(turretPos);
            turret2.setPosition(turretPos);
        }

        boolean overrideShoot = gamepad2.y;

        //TODO: ALL THE BUUUUTOOOOONS
        if (overrideShoot) {
            shooter.setStartIndexFromOrder(BALL_DESIRED_ORDER,GREEN_BALL_POS,POSITION_LIST[0]);
            intakeSubsystem.forceChambersFull();
            shooter.forceReady();
        }

        if (gamepad1.dpad_up) {
            SHOOTER_TICK_SPEED = 1415;
        }
        if (gamepad1.dpad_down) {
            SHOOTER_TICK_SPEED = 967;
        }
        if (gamepad1.dpad_right) {
            SHOOTER_TICK_SPEED = 1200;
        }
        if (gamepad1.dpad_left) {
            SHOOTER_TICK_SPEED = 210;

        }


        if (gamepad2.dpad_up) {
            TURRET_HOME = 0.5;
        }
        if (gamepad2.dpad_left) {
            TURRET_HOME = 0.605;
        }
        if (gamepad2.dpad_right) {
            TURRET_HOME = 0.395;
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

        if (gamepad2.left_bumper) {
            shooter.cancelShoot();
        }
        else if (gamepad2.a) BALL_DESIRED_ORDER = 67;
        else if (gamepad2.b) BALL_DESIRED_ORDER = 2;
        else if (gamepad2.x) BALL_DESIRED_ORDER = 3;

        if (shooter.isIdle()) {
            shooter.setStartIndexFromOrder(
                    BALL_DESIRED_ORDER,
                    GREEN_BALL_POS,
                    POSITION_LIST[0]
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
            double REAL_SHOOT_SPEED = SHOOTER_TICK_SPEED/(1+Math.pow(2, -2.67 * shootingAccelTimer.seconds()));
            shoot1.setVelocity(REAL_SHOOT_SPEED);
            shoot2.setVelocity(REAL_SHOOT_SPEED);
        } else {
            shoot1.setVelocity(0);
            shoot2.setVelocity(0);
        }


        telemetry.addData("Turret Pos", turretPos);


        telemetry.addData("Intake State", intakeSubsystem.getState());
        telemetry.addData("Next Spinner Index", intakeSubsystem.getNextSpinnerIndex());
        telemetry.addData("Shooter State", shooter.getState());

        telemetry.addData("pusherPos", pusher.getPosition());
        telemetry.addData("spinnerPos", spinner.getPosition());
        telemetry.update();
    }
    public void smoothingTurretAccel() {
        shootingAccelTimer.reset();
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