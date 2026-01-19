package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "path_test", group = "Autonomous")
public class pathTesting extends LinearOpMode {
    double INTAKE_CREEP_SPEED = 0.167;

    private IntakeSubsystem intakeSubsystem;
    private colorSensorDecode colorBench;

    private colorSensorDecode.DetectedColor c2, c3, c4;

    private Shooter shooter;
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher, turret1, turret2;

    private double PUSHER_UP = 0.55;
    private double PUSHER_DOWN = 0.2467;

    private int DESIRED_ORDER = 1;

    private CameraSubsystem camera;

    private int SHOOTER_TICK_SPEED = 1150;


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-50.75,50.25,Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // --------
        // POS LIST
        // --------
        final double[] POSITION_LIST = {
                0.882,
                0.7367,
                0.5885,
                0.438,
                0.2732,
                0.136,
                0
        };

        colorBench = new colorSensorDecode();
        colorBench.init(hardwareMap);

        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.init(hardwareMap);

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");

        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        pusher.setPosition(PUSHER_DOWN);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

        shoot1.setVelocityPIDFCoefficients(
                1,  // kP
                0.0,   // kI
                0.0,   // kD
                15.5   // kF (approximate)
        );

        shoot2.setVelocityPIDFCoefficients(
                1,
                0.0,
                0.0,
                15.5
        );

        camera = new CameraSubsystem();
        camera.init(hardwareMap);
        camera.setMode(CameraSubsystem.Mode.AUTO);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = new Shooter(
                shoot1,
                shoot2,
                spinner,
                pusher,
                POSITION_LIST
        );

        Action trajectory1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(
                        new Pose2d(-25, 25, Math.toRadians(90)),
                        Math.toRadians(0)
                )
                .build();

        Action trajectory2 = drive.actionBuilder(
                        new Pose2d(-25, 25, Math.toRadians(90))
                )
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(-12, 34),
                        Math.toRadians(90)
                )
                .build();

        Action trajectory3 = drive.actionBuilder(
                        new Pose2d(-12, 45, Math.toRadians(90))
                )
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(-25, 25),
                        Math.toRadians(180)
                )
                .build();
        Action trajectory4 = drive.actionBuilder(
                    new Pose2d(-25,25,Math.toRadians(90))
                )
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(12,34),
                        Math.toRadians(90)
                )
                .build();
        Action trajectory5 = drive.actionBuilder(
                        new Pose2d(12, 45, Math.toRadians(90))
                )
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(-25, 25),
                        Math.toRadians(180)
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;


        ElapsedTime scanTimer = new ElapsedTime();
        scanTimer.reset();

        while (opModeIsActive() && scanTimer.seconds() < 1.5) {
            camera.update();
            if (camera.hasOrderTarget()) {
                DESIRED_ORDER = camera.getDesiredOrderFromTag();
            }
            telemetry.addData("Scanning AprilTag...", scanTimer.seconds());
            telemetry.addData("Detected Order", DESIRED_ORDER);
            telemetry.update();

            sleep(20);
        }

        int FINAL_ORDER = DESIRED_ORDER;

        shooter.startSpinning(1167);

        shooter.goToNextPosition();
        shooter.forceReady();
        shooter.update(false, false);
        //TODO: FUCKING CHANGE THE 67's
        shooter.setStartIndexFromOrder(FINAL_ORDER,1,POSITION_LIST[0]);

        turret1.setPosition(0.8);
        turret2.setPosition(0.8);


// --------------------
// SHOOT PRELOAD BALLS
// --------------------


// Spin shooter wheels

        shooter.goToNextPosition();

        //GO TO SHOOTING LOCATION
        Actions.runBlocking(trajectory1);

        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

// Keep shooting until shooter FSM says NO_BALLS
        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 6.5) {

            shooter.update(
                    true,   // shootPressed = true
                    true    // intakeFull = true (we started with balls)
            );

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }


        //GO TO INTAKE LOCATION
        intakeSubsystem.forceIntakeReady();
        spinner.setPosition(POSITION_LIST[0]);
        Actions.runBlocking(trajectory2);

        // --------------------
        // INTAKE START
        // --------------------
        intakeSubsystem.startIntake();
        intakeSubsystem.forceIntakeReady();
        shooter.update(false, false);

        // --------------------
        // CREEP FORWARD (CONSTANT SPEED)
        // --------------------
        ElapsedTime creepTimer = new ElapsedTime();
        creepTimer.reset();

        while (opModeIsActive()
                && creepTimer.seconds() < 3.5) {

            // Drive forward
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(INTAKE_CREEP_SPEED, 0.0),
                            0.0
                    )
            );
            drive.updatePoseEstimate();

            // Read sensors
            c2 = colorBench.getDetectedColor(1, telemetry);
            c3 = colorBench.getDetectedColor(2, telemetry);
            c4 = colorBench.getDetectedColor(3, telemetry);

            // Update intake FSM
            intakeSubsystem.update(
                    c2,
                    c3,
                    c4,
                    shooter.getState()
            );

            int spinnerIndex = intakeSubsystem.getNextSpinnerIndex();
            spinnerIndex = Math.max(0, Math.min(spinnerIndex, POSITION_LIST.length - 1));
            spinner.setPosition(POSITION_LIST[spinnerIndex]);

            shooter.update(false, false);

            telemetry.addData("Spinner Index", spinnerIndex);
            telemetry.addData("Intake State", intakeSubsystem.getState());
            telemetry.update();

            idle();
        }


// stop
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0.0, 0.0),
                        0.0
                )
        );


        // --------------------
        // STOP INTAKE
        // --------------------
        intakeSubsystem.stopIntake();

        intakeSubsystem.reverseIntake();

        //go back to shooting place
        shooter.setStartIndexFromOrder(FINAL_ORDER, 3,POSITION_LIST[0]); // or real detected order
        shooter.goToNextPosition();
        Actions.runBlocking(trajectory3);
        intakeSubsystem.stopIntake();
        //TODO: FUCKING CHANGE THE 67's
        shooter.forceReady();
        shooter.update(false, true);

        shootTimer.reset();

// Spin shooter wheels
//        shoot1.setVelocity(1200);
//        shoot2.setVelocity(1200);



        //GO TO SHOOTING LOCATION
        //turret1.setPosition(0.69);
//        turret2.setPosition(0.69);
        turret1.setPosition(0.8);
        turret2.setPosition(0.8);

// Keep shooting until shooter FSM says NO_BALLS
        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 5.75) {
            boolean allowShoot = shootTimer.seconds() >= 0.75;
            shooter.update(
                    allowShoot,   // shootPressed = true
                    true    // intakeFull = true (we started with balls)
            );

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }


        //GO TO INTAKE LOCATION
        intakeSubsystem.forceIntakeReady();
        spinner.setPosition(POSITION_LIST[0]);
        Actions.runBlocking(trajectory4);

        // --------------------
        // INTAKE START
        // --------------------
        intakeSubsystem.startIntake();
        intakeSubsystem.forceIntakeReady();
        shooter.update(false, false);

        // --------------------
        // CREEP FORWARD (CONSTANT SPEED)
        // --------------------
        creepTimer.reset();

        while (opModeIsActive()
                && creepTimer.seconds() < 3.5) {

            // Drive forward
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(INTAKE_CREEP_SPEED, 0.0),
                            0.0
                    )
            );
            drive.updatePoseEstimate();

            // Read sensors
            c2 = colorBench.getDetectedColor(1, telemetry);
            c3 = colorBench.getDetectedColor(2, telemetry);
            c4 = colorBench.getDetectedColor(3, telemetry);

            // Update intake FSM
            intakeSubsystem.update(
                    c2,
                    c3,
                    c4,
                    shooter.getState()
            );

            int spinnerIndex = intakeSubsystem.getNextSpinnerIndex();
            spinnerIndex = Math.max(0, Math.min(spinnerIndex, POSITION_LIST.length - 1));
            spinner.setPosition(POSITION_LIST[spinnerIndex]);

            shooter.update(false, false);

            telemetry.addData("Spinner Index", spinnerIndex);
            telemetry.addData("Intake State", intakeSubsystem.getState());
            telemetry.update();

            idle();
        }


// stop
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0.0, 0.0),
                        0.0
                )
        );


        // --------------------
        // STOP INTAKE
        // --------------------
        intakeSubsystem.stopIntake();

        intakeSubsystem.reverseIntake();

        //go back to shooting place
        shooter.setStartIndexFromOrder(FINAL_ORDER, 2,POSITION_LIST[0]); // or real detected order
        shooter.goToNextPosition();
        Actions.runBlocking(trajectory5);
        intakeSubsystem.stopIntake();
        //TODO: FUCKING CHANGE THE 67's
        shooter.forceReady();
        shooter.update(false, true);

        shootTimer.reset();

// Spin shooter wheels
//        shoot1.setVelocity(1200);
//        shoot2.setVelocity(1200);



        //GO TO SHOOTING LOCATION
        //turret1.setPosition(0.69);
//        turret2.setPosition(0.69);
        turret1.setPosition(0.8);
        turret2.setPosition(0.8);

// Keep shooting until shooter FSM says NO_BALLS
        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 5.75) {
            boolean allowShoot = shootTimer.seconds() >= 0.75;
            shooter.update(
                    allowShoot,   // shootPressed = true
                    true    // intakeFull = true (we started with balls)
            );

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }
    }
}
