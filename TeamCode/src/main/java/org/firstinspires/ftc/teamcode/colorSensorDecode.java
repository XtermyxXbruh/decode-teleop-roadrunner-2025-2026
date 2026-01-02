package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensorDecode {

    private static final int SENSOR_COUNT = 4;

    private NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[SENSOR_COUNT];
    private DistanceSensor[] distanceSensors = new DistanceSensor[SENSOR_COUNT];

    // ---------- CONSTANTS ----------
    private static final double MAX_CHAMBER_DISTANCE_CM = 8.0;

    // Intake brightness threshold
    private static final double INTAKE_BRIGHTNESS_MAX = 1.25;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // ==============================
    // INIT
    // ==============================
    public void init(HardwareMap hwMap) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            colorSensors[i] =
                    hwMap.get(NormalizedColorSensor.class, "colorSensor" + (i + 1));

            colorSensors[i].setGain(24);
            distanceSensors[i] = (DistanceSensor) colorSensors[i];
        }
    }

    // ==============================
    // MAIN DETECTION
    // ==============================
    public DetectedColor getDetectedColor(int sensorIndex, Telemetry telemetry) {

        // --------------------------
        // READ COLOR
        // --------------------------
        NormalizedRGBA colors = colorSensors[sensorIndex].getNormalizedColors();

        double alpha = Math.max(colors.alpha, 0.001);
        double r = colors.red   / alpha;
        double g = colors.green / alpha;
        double b = colors.blue  / alpha;

        telemetry.addData("S" + sensorIndex + " R", "%.3f", r);
        telemetry.addData("S" + sensorIndex + " G", "%.3f", g);
        telemetry.addData("S" + sensorIndex + " B", "%.3f", b);

// --------------------------
// SENSOR 0 (INTAKE / CHAMBER 4 STYLE)
// --------------------------
        if (sensorIndex == 0) {

            double max = Math.max(r, Math.max(g, b));
            double min = Math.min(r, Math.min(g, b));
            double spread = max - min;

            telemetry.addData("S0 Spread", spread);

            // ---------- BALL PRESENCE ----------
            // Flat readings = empty
            if (spread < 0.18) {
                return DetectedColor.UNKNOWN;
            }

            // ---------- PURPLE ----------
            // Purple = blue dominant
            if (b > g + 0.05 && b > r + 0.08) {
                return DetectedColor.PURPLE;
            }

            // ---------- GREEN ----------
            // Green = green dominant (allow high brightness)
            if (g > r + 0.07 && g > b + 0.03) {
                return DetectedColor.GREEN;
            }

            return DetectedColor.UNKNOWN;
        }

        // =====================================================
        // SENSOR 4 (index 3) — intake-like but DIFFERENT
        // =====================================================
        if (sensorIndex == 3) {

            // Reject empty space (weak green)
            if (g < 0.28) {
                return DetectedColor.UNKNOWN;
            }

            // Green ball
            if (g > b + 0.05 && g > r + 0.10) {
                return DetectedColor.GREEN;
            }

            // Purple ball
            if (b > g && b > r) {
                return DetectedColor.PURPLE;
            }

            return DetectedColor.UNKNOWN;
        }

        // =====================================================
        // CHAMBER SENSORS (index 1 & 2)
        // =====================================================
        double distanceCm =
                distanceSensors[sensorIndex].getDistance(DistanceUnit.CM);

        telemetry.addData("S" + sensorIndex + " Dist (cm)", "%.2f", distanceCm);

        if (!Double.isFinite(distanceCm) || distanceCm > MAX_CHAMBER_DISTANCE_CM) {
            return DetectedColor.UNKNOWN;
        }

        // Chamber GREEN
        if (g > 0.28 &&
                g > b + 0.05 &&
                g > r + 0.10) {

            return DetectedColor.GREEN;
        }

        // Chamber PURPLE
        if (b > 0.32 &&
                b > g + 0.05 &&
                b > r + 0.10) {

            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
}
