package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoRoutine;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.colorSensorDecode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;

public class IntakeCreepRoutine implements AutoRoutine {

    private final MecanumDrive drive;
    private final IntakeSubsystem intake;
    private final colorSensorDecode colors;
    private final Shooter shooter;

    private final double speed;
    private final double duration;

    private final ElapsedTime timer = new ElapsedTime();

    public IntakeCreepRoutine(
            MecanumDrive drive,
            IntakeSubsystem intake,
            colorSensorDecode colors,
            Shooter shooter,
            double speed,
            double durationSeconds
    ) {
        this.drive = drive;
        this.intake = intake;
        this.colors = colors;
        this.shooter = shooter;
        this.speed = speed;
        this.duration = durationSeconds;
    }

    @Override
    public void start() {
        timer.reset();
        intake.startIntake();
        intake.forceIntakeReady();
    }

    @Override
    public void update() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(speed, 0),
                        0
                )
        );
        drive.updatePoseEstimate();

        intake.update(
                colors.getDetectedColor(1, null),
                colors.getDetectedColor(2, null),
                colors.getDetectedColor(3, null),
                shooter.getState()
        );
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= duration;
    }
}
