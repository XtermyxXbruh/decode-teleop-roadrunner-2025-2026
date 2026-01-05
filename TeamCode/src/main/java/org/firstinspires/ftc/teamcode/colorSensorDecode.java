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

    // =========================
    // MEMORY (NEW)
    // =========================
    private DetectedColor[] lastSeenColor = {
            DetectedColor.UNKNOWN,
            DetectedColor.UNKNOWN,
            DetectedColor.UNKNOWN,
            DetectedColor.UNKNOWN
    };

    // ---------- CONSTANTS ----------
    private static final double MAX_CHAMBER_DISTANCE_CM = 8.0;

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
    // LAST SEEN COLOR (NEW API)
    // ==============================
    public DetectedColor getLastSeenColor(int sensorIndex) {
        return lastSeenColor[sensorIndex];
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

        DetectedColor detected = DetectedColor.UNKNOWN;

        // ============================
        // SENSOR 0 (INTAKE / SPECIAL)
        // ============================
        if (sensorIndex == 0) {

            double max = Math.max(r, Math.max(g, b));
            double min = Math.min(r, Math.min(g, b));
            double spread = max - min;

            telemetry.addData("S0 Spread", "%.3f", spread);

            if (spread < 0.18) {
                detected = DetectedColor.UNKNOWN;
            } else if (b > g + 0.05 && b > r + 0.08) {
                detected = DetectedColor.PURPLE;
            } else if (g > r + 0.07 && g > b + 0.03) {
                detected = DetectedColor.GREEN;
            }
        }

        // ============================
        // SENSOR 3 (INTAKE-LIKE)
        // ============================
        else if (sensorIndex == 3) {

            if (g < 0.28) {
                detected = DetectedColor.UNKNOWN;
            } else if (g > b + 0.05 && g > r + 0.10) {
                detected = DetectedColor.GREEN;
            } else if (b > g && b > r) {
                detected = DetectedColor.PURPLE;
            }
        }

        // ============================
        // CHAMBER SENSORS (1 & 2)
        // ============================
        else {
            double distanceCm =
                    distanceSensors[sensorIndex].getDistance(DistanceUnit.CM);

            telemetry.addData("S" + sensorIndex + " Dist (cm)", "%.2f", distanceCm);

            if (Double.isFinite(distanceCm) &&
                    distanceCm <= MAX_CHAMBER_DISTANCE_CM) {

                if (g > 0.28 && g > b + 0.05 && g > r + 0.10) {
                    detected = DetectedColor.GREEN;
                } else if (b > 0.32 && b > g + 0.05 && b > r + 0.10) {
                    detected = DetectedColor.PURPLE;
                }
            }
        }

        // ============================
        // MEMORY UPDATE (IMPORTANT)
        // ============================
        if (detected != DetectedColor.UNKNOWN) {
            lastSeenColor[sensorIndex] = detected;
        }

        return detected;
    }
}
