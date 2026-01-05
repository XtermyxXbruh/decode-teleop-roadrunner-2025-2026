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

import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.colorSensorDecode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "chitty_kitty", group = "Autonomous")
public class chit_chat_auto extends LinearOpMode {
    double INTAKE_CREEP_SPEED = 0.15;

    private IntakeSubsystem intakeSubsystem;
    private colorSensorDecode colorBench;

    private colorSensorDecode.DetectedColor c2, c3, c4;

    private Shooter shooter;
    private DcMotorEx shoot1, shoot2;
    private Servo spinner, pusher;


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-63, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        colorBench = new colorSensorDecode();
        colorBench.init(hardwareMap);

        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.init(hardwareMap);

        intakeSubsystem.forceIntakeReady();

        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");

        spinner = hardwareMap.get(Servo.class, "spinner");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

// SAME POSITION LIST YOU USE IN TELEOP
        double[] POSITION_LIST = {
                0.985, 0.94,
                0.853, 0.79,
                0.6877, 0.64,
                0.5567, 0.49,
                0.41, 0.34,
                0.2467, 0.17,
                0.1, 0.2,
                0
        };

        shooter = new Shooter(
                shoot1,
                shoot2,
                spinner,
                pusher,
                POSITION_LIST
        );

        Action testtrajec = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToConstantHeading(
                        new Vector2d(-12, 20),
                        Math.toRadians(90)
                )
                .build();

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        if (isStopRequested()) return;

        // --------------------
        // PATH TO INTAKE AREA
        // --------------------
        Actions.runBlocking(testtrajec);

        // --------------------
        // INTAKE START
        // --------------------
        intakeSubsystem.startIntake();


        // --------------------
        // CREEP FORWARD (CONSTANT SPEED)
        // --------------------
        ElapsedTime creepTimer = new ElapsedTime();
        creepTimer.reset();

        while (opModeIsActive()
                && creepTimer.seconds() < 5
                && !intakeSubsystem.chambersFull()) {

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

            // 🔥 THIS IS THE MISSING PIECE
            int spinnerIndex = intakeSubsystem.getNextSpinnerIndex();
            spinnerIndex = Math.max(0, Math.min(spinnerIndex, POSITION_LIST.length - 1));
            spinner.setPosition(POSITION_LIST[spinnerIndex]);

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

    }
}
