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


@Autonomous(name = "B-FAR-6b-0s", group = "Autonomous")
public class blue_far_six_zero extends LinearOpMode {
    double INTAKE_CREEP_SPEED = 0.15;

    private IntakeSubsystem intakeSubsystem;
    private colorSensorDecode colorBench;

    private colorSensorDecode.DetectedColor c2, c3, c4;

    private Shooter shooter;
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher, turret1, turret2;

    private double PUSHER_UP = 0.4;
    private double PUSHER_DOWN = 0.06;

    private double SHOOTER_TICK_SPEED = 1426.7;


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(63, -9, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // --------
        // POS LIST
        // --------
        final double[] POSITION_LIST = {
                0.986, 0.945,
                0.855,  0.781,
                0.7,  0.633,
                0.54,  0.485,
                0.4,  0.322,
                0.2467,0.17,
                0.1,   0.2,
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

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        shooter = new Shooter(
                shoot1,
                shoot2,
                spinner,
                pusher,
                POSITION_LIST
        );
        //TODO: FUCKING CHANGE THE 67's
        shooter.setStartIndexFromOrder(67,67,spinner.getPosition());
        shooter.forceReady();
        shooter.update(false, false);

        Action trajectory1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36,-33),Math.toRadians(270))
                .build();

        Action trajectory2 = drive.actionBuilder(new Pose2d(36,-33,Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(63,-9),Math.toRadians(0))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        // --------------------
// SHOOT PRELOAD BALLS
// --------------------
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

// Spin shooter wheels
        shoot1.setVelocity(SHOOTER_TICK_SPEED);
        shoot2.setVelocity(SHOOTER_TICK_SPEED);

        spinner.setPosition(POSITION_LIST[0]);

        //GO TO SHOOTING LOCATION
        turret1.setPosition(0.21);
        turret2.setPosition(0.21);

// Keep shooting until shooter FSM says NO_BALLS
        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 13.5) {
            boolean allowShoot = shootTimer.seconds() >= 1.5;
            shooter.update(
                    allowShoot,   // shootPressed = true
                    true    // intakeFull = true (we started with balls)
            );

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }

// Stop shooter wheels
        shoot1.setVelocity(0);
        shoot2.setVelocity(0);

        //GO TO INTAKE LOCATION
        intakeSubsystem.forceIntakeReady();
        spinner.setPosition(POSITION_LIST[0]);
        Actions.runBlocking(trajectory1);

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
                && creepTimer.seconds() < 4) {

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
        Actions.runBlocking(trajectory2);
        intakeSubsystem.stopIntake();
        spinner.setPosition(POSITION_LIST[0]);
        //TODO: FUCKING CHANGE THE 67's
        shooter.setStartIndexFromOrder(67, 67,spinner.getPosition()); // or real detected order
        shooter.forceReady();
        shooter.update(false, true);

        shootTimer.reset();

// Spin shooter wheels
        shoot1.setVelocity(SHOOTER_TICK_SPEED);
        shoot2.setVelocity(SHOOTER_TICK_SPEED);



        //GO TO SHOOTING LOCATION
        turret1.setPosition(0.21);

        turret2.setPosition(0.21);

// Keep shooting until shooter FSM says NO_BALLS
        while (opModeIsActive()
                && shooter.getState() != Shooter.State.NO_BALLS
                && shootTimer.seconds() <= 13.5) {
            boolean allowShoot = shootTimer.seconds() >= 1.5;
            shooter.update(
                    allowShoot,   // shootPressed = true
                    true    // intakeFull = true (we started with balls)
            );

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.update();
            idle();
        }

// Stop shooter wheels
        shoot1.setVelocity(0);
        shoot2.setVelocity(0);

        Actions.runBlocking(trajectory1);
    }
}
