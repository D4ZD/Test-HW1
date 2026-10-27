package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {
    private static final double EPSILON = 1e-9;
    private static final double ABS_SPEED_KMH = 31.0;
    private static final double ABS_TRIGGER_PRESSURE = 81.0;

    @ParameterizedTest
    @CsvFileSource(resources = "/autopilot/core/brake-cases.csv", numLinesToSkip = 1)
    void apply_singleRequest_returnsExpectedPressure(double requestedPressure, double speedKmh,
                                                    GearBox.Gear gear, double expectedPressure) {
        BrakeController brakes = new BrakeController();

        assertEquals(expectedPressure, brakes.apply(requestedPressure, speedKmh, gear), EPSILON);
    }

    @Test
    void apply_consecutiveAbsRequests_alternatesHighLowHigh() {
        BrakeController brakes = new BrakeController();

        double first = brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);
        double second = brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);
        double third = brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);

        assertAll(
                () -> assertEquals(90.0, first),
                () -> assertEquals(60.0, second),
                () -> assertEquals(90.0, third)
        );
    }

    @Test
    void apply_regularPressureBetweenAbsRequests_resetsPulseToHigh() {
        BrakeController brakes = new BrakeController();
        brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);
        brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);
        brakes.apply(80.0, ABS_SPEED_KMH, GearBox.Gear.D);

        double result = brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);

        assertEquals(90.0, result);
    }

    @Test
    void apply_whileInPark_throwsIllegalStateException() {
        BrakeController brakes = new BrakeController();

        assertThrows(IllegalStateException.class,
                () -> brakes.apply(50.0, 20.0, GearBox.Gear.P));
    }

    @Test
    void apply_withNullGear_throwsNullPointerException() {
        BrakeController brakes = new BrakeController();

        assertThrows(NullPointerException.class,
                () -> brakes.apply(50.0, 20.0, null));
    }

    @Test
    void apply_withNegativePressure_throwsIllegalArgumentException() {
        BrakeController brakes = new BrakeController();

        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(-0.01, 20.0, GearBox.Gear.D));
    }

    @Test
    void apply_withNonFinitePressure_throwsIllegalArgumentException() {
        BrakeController brakes = new BrakeController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(Double.NaN, 20.0, GearBox.Gear.D)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(Double.POSITIVE_INFINITY, 20.0, GearBox.Gear.D)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(Double.NEGATIVE_INFINITY, 20.0, GearBox.Gear.D))
        );
    }

    @Test
    void apply_withNegativeSpeed_throwsIllegalArgumentException() {
        BrakeController brakes = new BrakeController();

        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(50.0, -1.0, GearBox.Gear.D));
    }

    @Test
    void apply_withNonFiniteSpeed_throwsIllegalArgumentException() {
        BrakeController brakes = new BrakeController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(50.0, Double.NaN, GearBox.Gear.D)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(50.0, Double.POSITIVE_INFINITY, GearBox.Gear.D)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> brakes.apply(50.0, Double.NEGATIVE_INFINITY, GearBox.Gear.D))
        );
    }

    @Test
    void emergencyBrake_inDrive_setsFullPressureAndWarning() {
        BrakeController brakes = new BrakeController();

        brakes.emergencyBrake(GearBox.Gear.D);

        assertAll(
                () -> assertEquals(100.0, brakes.getPressure()),
                () -> assertTrue(brakes.isWarningActive())
        );
    }

    @Test
    void emergencyBrake_whileInPark_throwsIllegalStateException() {
        BrakeController brakes = new BrakeController();

        assertThrows(IllegalStateException.class, () -> brakes.emergencyBrake(GearBox.Gear.P));
    }

    @Test
    void emergencyBrake_withNullGear_throwsNullPointerException() {
        BrakeController brakes = new BrakeController();

        assertThrows(NullPointerException.class, () -> brakes.emergencyBrake(null));
    }

    @Test
    void emergencyBrake_afterAbsPulse_resetsNextAbsPulseToHigh() {
        BrakeController brakes = new BrakeController();
        brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);
        brakes.emergencyBrake(GearBox.Gear.D);

        double result = brakes.apply(ABS_TRIGGER_PRESSURE, ABS_SPEED_KMH, GearBox.Gear.D);

        assertEquals(90.0, result);
    }
}
