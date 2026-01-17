package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Shooter;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    ElapsedTime intakeTimer = new ElapsedTime();
    // =====================
    // INTAKE STATES
    // =====================
    public enum IntakeState {
        BEFORE_START,
        READY,
        INTAKE_DETECT,
        WAIT_FOR_READY,
        CHAMBERS_FULL
    }

    private IntakeState intakeState = IntakeState.BEFORE_START;

    private int nextSpinnerIndex = 0;

    private DcMotor intakeMotor;

    // =====================
    // INIT
    // =====================
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(0);

        intakeState = IntakeState.BEFORE_START;
        nextSpinnerIndex = 0;
    }

    // =====================
    // UPDATE (CALLED EVERY LOOP)
    // =====================
    public void update(
            colorSensorDecode.DetectedColor c2,
            colorSensorDecode.DetectedColor c3,
            colorSensorDecode.DetectedColor c4,
            Shooter.State shootingState   // keep generic so no circular dependency
    ) {

        switch (intakeState) {

            case BEFORE_START:
                if (intakeTimer.seconds() >= 1) {
                    nextSpinnerIndex = 0;
                    intakeState = IntakeState.READY;
                }
                break;

            case READY:
                if (intakeTimer.seconds() >= 0.2 && (c4 == colorSensorDecode.DetectedColor.GREEN ||
                        c4 == colorSensorDecode.DetectedColor.PURPLE)) {

                    nextSpinnerIndex++;
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_DETECT;
                }
                break;

            case INTAKE_DETECT:
                if (intakeTimer.seconds() >= 0.25) {
                    intakeState = IntakeState.WAIT_FOR_READY;
                }
                break;

            case WAIT_FOR_READY:
                if (intakeTimer.seconds() >= 0.5) {
                    if (nextSpinnerIndex >= 3) {
                        intakeState = IntakeState.CHAMBERS_FULL;
                    } else {
                        intakeState = IntakeState.READY;
                    }
                }
                break;

            case CHAMBERS_FULL:
                intakeTimer.reset();
                if (shootingState == Shooter.State.NO_BALLS) {
                    nextSpinnerIndex = 0;
                    intakeState = IntakeState.BEFORE_START;
                }
                break;

        }
    }

    // =====================
    // GETTERS
    // =====================
    public int getNextSpinnerIndex() {
        return nextSpinnerIndex;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public boolean chambersFull() {
        return intakeState == IntakeState.CHAMBERS_FULL;
    }

    public void forceChambersFull() {
        intakeState = IntakeState.CHAMBERS_FULL;
        intakeTimer.reset();
    }

    public void forceIntakeReady() {
        intakeState = IntakeState.BEFORE_START;
    }

    public void startIntake() {
        intakeMotor.setPower(1.0);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    public void reverseIntake() {
        intakeMotor.setPower(-1);
    }
}
