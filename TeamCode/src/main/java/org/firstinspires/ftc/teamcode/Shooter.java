package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    public enum State {
        READY,
        PUSHER_UP,
        WAIT_FOR_SPIN,
        PUSHER_DOWN,
        SPINNER_FULL,
        ABORT,
        NO_BALLS
    }

    private State state = State.NO_BALLS;

    private final DcMotorEx shooter1, shooter2;
    private final Servo spinner, pusher;
    private final ElapsedTime timer = new ElapsedTime();

    private final double[] positions;

    private static final double PUSHER_UP = 0.55;
    private static final double PUSHER_DOWN = 0.25;

    private int startShootPos = 0;
    private int nextShootPos = 0;

    private double waitTime = 0;

    // Shooter wheel control
    private double targetVelocity = 0;
    private boolean spinning = false;

    // PIDF defaults
    private static final double DEFAULT_KP = 1.0;
    private static final double DEFAULT_KF = 15.5;

    // =====================
    // CONSTRUCTOR
    // =====================
    public Shooter(
            DcMotorEx shooter1,
            DcMotorEx shooter2,
            Servo spinner,
            Servo pusher,
            double[] positions
    ) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.spinner = spinner;
        this.pusher = pusher;
        this.positions = positions;

        // Motor configuration (LEGAL HERE)
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setVelocityPIDFCoefficients(DEFAULT_KP, 0, 0, DEFAULT_KF);
        shooter2.setVelocityPIDFCoefficients(DEFAULT_KP, 0, 0, DEFAULT_KF);
    }

    // =====================
    // MAIN UPDATE (CALL EVERY LOOP)
    // =====================
    public void update(boolean shootPressed, boolean intakeFull) {

        // Wheel control
        if (spinning) {
            double error = targetVelocity - shooter1.getVelocity();
            if (error > 42) {
                shooter1.setVelocity(targetVelocity+250);
                shooter2.setVelocity(targetVelocity+250);
            }else {
                shooter1.setVelocity(targetVelocity);
                shooter2.setVelocity(targetVelocity);
            }

        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        switch (state) {
            case READY:
                pusher.setPosition(PUSHER_DOWN);
                spinner.setPosition(positions[nextShootPos]);

                if (shootPressed) {
                    timer.reset();
                    state = State.PUSHER_UP;
                }
                break;

            case PUSHER_UP:
                if (timer.seconds() > 0.1 + waitTime) {
                    pusher.setPosition(PUSHER_UP);
                    nextShootPos++;
                    timer.reset();
                    state = State.WAIT_FOR_SPIN;
                }
                break;

            case WAIT_FOR_SPIN:
                if (timer.seconds() > 0.45 + waitTime) {
                    timer.reset();
                    state = State.PUSHER_DOWN;
                }
                break;

            case PUSHER_DOWN:
                pusher.setPosition(PUSHER_DOWN);
                if (timer.seconds() > 0.5 + waitTime) {
                    timer.reset();
                    state = State.SPINNER_FULL;
                }
                break;

            case SPINNER_FULL:
                spinner.setPosition(positions[nextShootPos]);
                if (timer.seconds() > 0.25 + waitTime) {
                    timer.reset();
                    if (nextShootPos >= startShootPos + 3) {
                        state = State.NO_BALLS;
                    } else {
                        state = State.PUSHER_UP;
                    }
                }
                break;

            case ABORT:
                pusher.setPosition(PUSHER_DOWN);
                if (timer.seconds() > 1) {
                    timer.reset();
                    nextShootPos = startShootPos;
                    state = State.NO_BALLS;
                }
                break;

            case NO_BALLS:
                pusher.setPosition(PUSHER_DOWN);
                if (intakeFull) {
                    state = State.READY;
                }
                break;
        }
    }

    // =====================
    // COMMAND API
    // =====================
    public void startSpinning(double velocity) {
        targetVelocity = velocity;
        spinning = true;
    }

    public void stopSpinning() {
        spinning = false;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void forceReady() {
        state = State.READY;
        timer.reset();
    }

    public boolean isIdle() {
        return state == State.NO_BALLS;
    }

    public State getState() {
        return state;
    }

    public void setWaitTime(double wait) {
        waitTime = wait;
    }

    public void setStartShootPos(int pos) {
        startShootPos = pos;
        nextShootPos = pos;
    }

    public int getNextShootPos() {
        return nextShootPos;
    }

    public void cancelShoot() {
        state = State.ABORT;
        timer.reset();
    }

    public void goToNextPosition() {
        spinner.setPosition(positions[nextShootPos]);
    }

    public void setStartIndexFromOrder(
            int desiredOrder,
            int greenBallPos,
            double latestServoPos
    ) {
        int trueBallPos = greenBallPos;

        if (desiredOrder == 67) startShootPos = 0;

        else if (desiredOrder == 1 && trueBallPos == 1) startShootPos = 0;
        else if (desiredOrder == 1 && trueBallPos == 2) startShootPos = 1;
        else if (desiredOrder == 1 && trueBallPos == 3) startShootPos = 2;

        else if (desiredOrder == 2 && trueBallPos == 1) startShootPos = 2;
        else if (desiredOrder == 2 && trueBallPos == 2) startShootPos = 0;
        else if (desiredOrder == 2 && trueBallPos == 3) startShootPos = 1;

        else if (desiredOrder == 3 && trueBallPos == 1) startShootPos = 1;
        else if (desiredOrder == 3 && trueBallPos == 2) startShootPos = 2;
        else if (desiredOrder == 3 && trueBallPos == 3) startShootPos = 0;

        nextShootPos = startShootPos;
    }

}
