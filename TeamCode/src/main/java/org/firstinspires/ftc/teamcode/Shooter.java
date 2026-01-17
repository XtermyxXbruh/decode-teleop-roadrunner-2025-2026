package org.firstinspires.ftc.teamcode;

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
    private final double PUSHER_UP;
    private final double PUSHER_DOWN;

    private int startShootPos = 0;
    private int nextShootPos = 0;

    // ✅ THIS CONSTRUCTOR HAS **5 ARGUMENTS**
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

        this.PUSHER_UP = 0.567;
        this.PUSHER_DOWN = 0.27167;
    }

    public void setStartShootPos(int pos) {
        startShootPos = pos;
        nextShootPos = pos;
    }

    public void update(boolean shootPressed, boolean intakeFull) {

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
                spinner.setPosition(positions[nextShootPos]);
                if (timer.seconds() > 0.05) {
                    pusher.setPosition(PUSHER_UP);
                    nextShootPos++;
                    timer.reset();
                    state = State.WAIT_FOR_SPIN;
                }
                break;

            case WAIT_FOR_SPIN:
                pusher.setPosition(PUSHER_UP);
                if (timer.seconds() > 0.3) {
                    timer.reset();
                    state = State.PUSHER_DOWN;
                }
                break;

            case PUSHER_DOWN:
                pusher.setPosition(PUSHER_DOWN);
                if (timer.seconds() > 0.4) {
                    timer.reset();
                    state = State.SPINNER_FULL;
                }
                break;

            case SPINNER_FULL:
                spinner.setPosition(positions[nextShootPos]);
                if (timer.seconds() > 0.2) {
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
            case NO_BALLS:
                if (timer.seconds() >= 0.25) {
                    nextShootPos = startShootPos;
                    pusher.setPosition(PUSHER_DOWN);
                    if (intakeFull) {
                        state = State.READY;
                    }
                }
                break;
        }
    }


    public void setStartIndexFromOrder(int desiredOrder, int greenBallPos, double latestServoPos) {
        int trueBallPos = 1;
        if (latestServoPos == positions[0]) {
            if (greenBallPos == 1) {
                trueBallPos = 1;
            }else if (greenBallPos == 2) {
                trueBallPos = 2;
            }else if (greenBallPos == 3) {
                trueBallPos = 3;
            }else {
                trueBallPos = 1;
            }
        }else if (latestServoPos == positions[2]) {
            if (greenBallPos == 1) {
                trueBallPos = 3;
            }else if (greenBallPos == 2) {
                trueBallPos = 1;
            }else if (greenBallPos == 3) {
                trueBallPos = 2;
            }else {
                trueBallPos = 1;
            }
        }else if (latestServoPos == positions[4]) {
            if (greenBallPos == 1) {
                trueBallPos = 2;
            }else if (greenBallPos == 2) {
                trueBallPos = 3;
            }else if (greenBallPos == 3) {
                trueBallPos = 1;
            }else {
                trueBallPos = 1;
            }
        }
        if (desiredOrder == 67) startShootPos = 0;
        else if (desiredOrder == 1 && trueBallPos == 1) startShootPos = 0;
        else if (desiredOrder == 1 && trueBallPos == 2) startShootPos = 2;
        else if (desiredOrder == 1 && trueBallPos == 3) startShootPos = 1;

        else if (desiredOrder == 2 && trueBallPos == 1) startShootPos = 2;
        else if (desiredOrder == 2 && trueBallPos == 2) startShootPos = 1;
        else if (desiredOrder == 2 && trueBallPos == 3) startShootPos = 0;

        else if (desiredOrder == 3 && trueBallPos == 1) startShootPos = 1;
        else if (desiredOrder == 3 && trueBallPos == 2) startShootPos = 0;
        else if (desiredOrder == 3 && trueBallPos == 3) startShootPos = 2;

        nextShootPos = startShootPos;
    }


    public State getState() {
        return state;
    }

    public boolean isIdle() {
        return state == State.NO_BALLS;
    }

    public void forceReady() {
        state = State.READY;
        timer.reset();
    }

    public void goToNextPosition() {
        spinner.setPosition(positions[nextShootPos]);
    }

    public void cancelShoot() {
        if (state != State.NO_BALLS) {
            state = State.ABORT;
            timer.reset();
        }
    }

    public int getNextShootPos() { return nextShootPos; }
}
