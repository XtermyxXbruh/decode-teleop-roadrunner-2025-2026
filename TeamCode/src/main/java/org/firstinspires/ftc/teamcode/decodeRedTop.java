package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "decodeRedTop", group = "Autonomous")
public class decodeRedTop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-63,9,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        /*
        Action trajectory = drive.actionBuilder(initialPose)
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(-48, -24), 180)
                .build();

        Action trajectory2 = drive.actionBuilder(new Pose2d(-48,-24,Math.toRadians(270)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-20, 0, 200), 270)
                .build();

         */

        Action trajectory2 = drive.actionBuilder(new Pose2d(-12,33,Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-10,9),Math.toRadians(270))
                .build();



        waitForStart();
        if (isStopRequested()) return;

        //Actions.runBlocking(trajectory);
        //Actions.runBlocking(trajectory2);
        Actions.runBlocking(trajectory2);
    }
}
