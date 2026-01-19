package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "R-FAR-6b-0s", group = "Autonomous")
public class red_far_six_zero extends LinearOpMode {

    // --------------------
    // CONSTANTS
    // --------------------
    private static final double INTAKE_CREEP_SPEED = 0.167;
    private static final double SHOOTER_TICK_SPEED = 1430;

    private static final double PUSHER_DOWN = 0.23;

    // --------------------
    // SUBSYSTEMS
    // --------------------
    private IntakeSubsystem intakeSubsystem;
    private colorSensorDecode colorBench;
    private CameraSubsystem camera;
    private Shooter shooter;

    // --------------------
    // HARDWARE
    // --------------------
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher, turret1, turret2;

    private colorSensorDecode.DetectedColor c2, c3, c4;

    private int DESIRED_ORDER = 1;

    @Override
    public void runOpMode() {

        // --------------------
        // DRIVE SETUP
        // --------------------
        Pose2d initialPose = new Pose2d(63, 9, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // --------------------
        // SPINNER POSITIONS
        // --------------------
        final double[] POSITION_LIST = {
                0.882,
                0.7367,
                0.5885,
                0.438,
                0.2732,
                0.136,
                0
        };

        // --------------------
        // INIT SUBSYSTEMS
        // --------------------
        colorBench = new colorSensorDecode();
        colorBench.init(hardwareMap);

        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.init(hardwareMap);

        camera = new CameraSubsystem();
        camera.init(hardwareMap);
        camera.setMode(CameraSubsystem.Mode.AUTO);

        // --------------------
        // HARDWARE MAP
        // --------------------
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");

        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        pusher.setPosition(PUSHER_DOWN);

        // --------------------
        // SHOOTER FSM
        // --------------------
        shooter = new Shooter(
                shoot1,
                shoot2,
                spinner,
                pusher,
                POSITION_LIST
        );

        // --------------------
        // TRAJECTORIES
        // --------------------
        Action trajectory1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, 34), Math.toRadians(90))
                .build();

        Action trajectory2 = drive.actionBuilder(new Pose2d(36, 45, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(63, 9), Math.toRadians(0))
                .build();

        Action trajectory3 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, 34), Math.toRadians(90))
                .build();

        // --------------------
        // WAIT FOR START
        // --------------------
        waitForStart();
        if (isStopRequested()) return;

        // --------------------
        // APRILTAG SCAN
        // --------------------
        ElapsedTime scanTimer = new ElapsedTime();
        scanTimer.reset();

        while (opModeIsActive() && scanTimer.seconds() < 2) {
            camera.update();
            if (camera.hasOrderTarget()) {
                DESIRED_ORDER = camera.getDesiredOrderFromTag();
            }

            telemetry.addData("Scanning...", scanTimer.seconds());
            telemetry.addData("Detected Order", DESIRED_ORDER);
            telemetry.update();

            sleep(20);
        }

        int FINAL_ORDER = DESIRED_ORDER;

        // --------------------
        // PRELOAD SHOOT
        // --------------------
        shooter.startSpinning(SHOOTER_TICK_SPEED);
        shooter.setStartIndexFromOrder(FINAL_ORDER, 1, POSITION_LIST[0]);
        shooter.goToNextPosition();
        shooter.forceReady();
        shooter.update(false, false);

        turret1.setPosition(0.91);
        turret2.setPosition(0.91);

        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 7.5) {

            boolean allowShoot = shootTimer.seconds() >= 2.0;
            shooter.update(allowShoot, true);

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }

        shooter.stopSpinning();

        // --------------------
        // GO TO INTAKE
        // --------------------
        intakeSubsystem.forceIntakeReady();
        spinner.setPosition(POSITION_LIST[0]);
        Actions.runBlocking(trajectory1);

        intakeSubsystem.startIntake();

        ElapsedTime creepTimer = new ElapsedTime();
        creepTimer.reset();

        while (opModeIsActive() && creepTimer.seconds() < 4) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(INTAKE_CREEP_SPEED, 0.0),
                            0.0
                    )
            );
            drive.updatePoseEstimate();

            c2 = colorBench.getDetectedColor(1, telemetry);
            c3 = colorBench.getDetectedColor(2, telemetry);
            c4 = colorBench.getDetectedColor(3, telemetry);

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

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(0, 0), 0)
        );

        intakeSubsystem.stopIntake();
        intakeSubsystem.reverseIntake();

        // --------------------
        // RETURN + FINAL SHOOT
        // --------------------
        shooter.setStartIndexFromOrder(FINAL_ORDER, 1,POSITION_LIST[0]); // or real detected order
        shooter.goToNextPosition();
        Actions.runBlocking(trajectory2);

        intakeSubsystem.stopIntake();

        shooter.startSpinning(SHOOTER_TICK_SPEED);
        shooter.forceReady();
        shooter.update(false, true);

        shootTimer.reset();

        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 7.5) {

            boolean allowShoot = shootTimer.seconds() >= 2;
            shooter.update(allowShoot, true);

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }

        shooter.stopSpinning();



        Actions.runBlocking(trajectory3);

        shootTimer.reset();
        while (opModeIsActive()
                && shootTimer.seconds() < 1.5) {
            pusher.setPosition(PUSHER_DOWN);
            spinner.setPosition(POSITION_LIST[0]);
        }
    }
}
