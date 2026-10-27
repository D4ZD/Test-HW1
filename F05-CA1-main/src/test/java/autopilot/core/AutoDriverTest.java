package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AutoDriverTest {
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CURRENT_SPEED_KMH = 80.0;
    private static final double CLEAR_ROAD_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double OUTSIDE_LANE_THRESHOLD_METERS = 1.0;
    private static final double STEERING_CORRECTION_DEGREES = 45.0;
    private static final String SENSOR_FAILURE_MESSAGE = "sensor unavailable";

    private static final AutoDriver.DriveResult SAFE_MODE_RESULT = new AutoDriver.DriveResult(
            0.0, 30.0, AdaptiveCruise.Mode.BRAKE, false, true);

    @Test
    void tick_withNormalReadings_holdsSpeedWithoutBraking() {
        AutoDriver driver = createDriver(() -> CURRENT_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS, () -> CENTERED_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertEquals(new AutoDriver.DriveResult(
                0.0, 0.0, AdaptiveCruise.Mode.HOLD, false, false), result);
    }

    @Test
    void tick_withPositiveLaneDeviation_steersLeft() {
        AutoDriver driver = createDriver(() -> CURRENT_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS, () -> OUTSIDE_LANE_THRESHOLD_METERS);

        assertEquals(-STEERING_CORRECTION_DEGREES,
                driver.tick(GearBox.Gear.D).steeringAngleDegrees());
    }

    @Test
    void tick_withNegativeLaneDeviation_steersRight() {
        AutoDriver driver = createDriver(() -> CURRENT_SPEED_KMH,
                () -> CLEAR_ROAD_DISTANCE_METERS, () -> -OUTSIDE_LANE_THRESHOLD_METERS);

        assertEquals(STEERING_CORRECTION_DEGREES,
                driver.tick(GearBox.Gear.D).steeringAngleDegrees());
    }

    @Test
    void tick_whenSpeedSensorFails_entersSafeMode() {
        SpeedSensor failedSpeedSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        AutoDriver driver = createDriver(failedSpeedSensor,
                () -> CLEAR_ROAD_DISTANCE_METERS, () -> CENTERED_LANE_OFFSET_METERS);

        assertEquals(SAFE_MODE_RESULT, driver.tick(GearBox.Gear.D));
    }

    @Test
    void tick_whenObstacleSensorFails_entersSafeMode() {
        ObstacleSensor failedObstacleSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        AutoDriver driver = createDriver(() -> CURRENT_SPEED_KMH,
                failedObstacleSensor, () -> CENTERED_LANE_OFFSET_METERS);

        assertEquals(SAFE_MODE_RESULT, driver.tick(GearBox.Gear.D));
    }

    private static AutoDriver createDriver(SpeedSensor speed, ObstacleSensor obstacle, LaneSensor lane) {
        return new AutoDriver(speed, obstacle, lane, TARGET_SPEED_KMH);
    }
}
