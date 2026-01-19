package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoRoutine;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.colorSensorDecode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;

public class AutoShootRoutine implements AutoRoutine {

    private final Shooter shooter;
    private final DcMotorEx shoot1, shoot2;

    private final int targetVelo;
    private final double duration;

    private final ElapsedTime timer = new ElapsedTime();

    public AutoShootRoutine(
            Shooter shooter,
            DcMotorEx shoot1,
            DcMotorEx shoot2,
            int targetVelo,
            double durationSeconds
    ) {
        this.shooter = shooter;
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        this.targetVelo = targetVelo;
        this.duration = durationSeconds;
    }

    @Override
    public void start() {
        timer.reset();
        shooter.forceReady();
    }

    @Override
    public void update() {
        double measured =
                (shoot1.getVelocity() + shoot2.getVelocity()) / 2.0;

        int compensated = targetVelo;
        if (targetVelo - measured > 67) {
            compensated += 250;
        }

        shoot1.setVelocity(compensated);
        shoot2.setVelocity(compensated);

        shooter.update(true, true);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= duration;
    }
}
