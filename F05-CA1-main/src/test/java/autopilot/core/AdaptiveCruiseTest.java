package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AdaptiveCruiseTest {
    private static final double EPSILON = 1e-9;
    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double SAFE_DISTANCE_METERS = 40.0;

    @ParameterizedTest
    @CsvSource({
            "0, 0",
            "1, 0.5",
            "80, 40",
            "100.5, 50.25"
    })
    void safeDistanceFor_validSpeed_returnsHalfTheSpeed(double speedKmh, double expectedMeters) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(expectedMeters, cruise.safeDistanceFor(speedKmh), EPSILON);
    }

    @Test
    void safeDistanceFor_negativeSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(-1.0));
    }

    @Test
    void safeDistanceFor_nonFiniteSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.POSITIVE_INFINITY)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NEGATIVE_INFINITY))
        );
    }

    @ParameterizedTest(name = "{index}: target={0}, speed={1}, distance={2} => {3} {4}")
    @MethodSource("cruiseDecisions")
    void update_freshController_returnsExpectedDecision(
            double targetSpeedKmh, double speedKmh, double distanceMeters,
            AdaptiveCruise.Mode expectedMode, double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);

        AdaptiveCruise.Decision result = cruise.update(speedKmh, distanceMeters);

        assertAll(
                () -> assertEquals(expectedMode, result.mode()),
                () -> assertEquals(expectedAmount, result.amount(), EPSILON)
        );
    }

    private static Stream<Arguments> cruiseDecisions() {
        return Stream.of(
                Arguments.of(100.0, 80.0, 19.9, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(100.0, 80.0, 20.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(100.0, 80.0, 20.1, AdaptiveCruise.Mode.BRAKE, 49.75),
                Arguments.of(100.0, 80.0, 30.0, AdaptiveCruise.Mode.BRAKE, 25.0),
                Arguments.of(100.0, 80.0, 40.0, AdaptiveCruise.Mode.HOLD, 0.0),
                Arguments.of(100.0, 80.0, 40.1, AdaptiveCruise.Mode.ACCELERATE, 5.0),
                Arguments.of(100.0, 98.0, 100.0, AdaptiveCruise.Mode.ACCELERATE, 2.0),
                Arguments.of(100.0, 100.0, 100.0, AdaptiveCruise.Mode.HOLD, 0.0),
                Arguments.of(100.0, 120.0, 100.0, AdaptiveCruise.Mode.HOLD, 0.0),
                Arguments.of(100.0, 0.0, 0.0, AdaptiveCruise.Mode.EMERGENCY_BRAKE, 100.0),
                Arguments.of(100.0, 0.0, 1.0, AdaptiveCruise.Mode.ACCELERATE, 5.0),
                Arguments.of(0.0, 0.0, 1.0, AdaptiveCruise.Mode.HOLD, 0.0)
        );
    }

    @Test
    void update_afterBrakingAtSafeDistance_keepsMinimumBrakePressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 30.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.BRAKE, 10.0), result);
    }

    @Test
    void update_whileBrakingAtCloserDistance_usesProportionalPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 30.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, 35.0);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.BRAKE, 12.5), result);
    }

    @Test
    void update_belowReleaseDistance_keepsBraking() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 30.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, 47.9);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.BRAKE, 10.0), result);
    }

    @Test
    void update_atReleaseDistance_resumesAcceleration() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 30.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, 48.0);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.ACCELERATE, 5.0), result);
    }

    @Test
    void update_afterBrakingIsReleased_atSafeDistanceHoldsSpeed() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 30.0);
        cruise.update(CRUISING_SPEED_KMH, 48.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, SAFE_DISTANCE_METERS);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.HOLD, 0.0), result);
    }

    @Test
    void update_afterEmergencyBrake_belowReleaseDistanceStaysInBrakeMode() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(CRUISING_SPEED_KMH, 20.0);

        AdaptiveCruise.Decision result = cruise.update(CRUISING_SPEED_KMH, 40.0);

        assertEquals(new AdaptiveCruise.Decision(AdaptiveCruise.Mode.BRAKE, 10.0), result);
    }

    @Test
    void update_negativeSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(-1.0, 50.0));
    }

    @Test
    void update_nonFiniteSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.NaN, 50.0)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.POSITIVE_INFINITY, 50.0)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.NEGATIVE_INFINITY, 50.0))
        );
    }

    @Test
    void update_negativeDistance_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(CRUISING_SPEED_KMH, -1.0));
    }

    @Test
    void update_nonFiniteDistance_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(80.0, Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(80.0, Double.POSITIVE_INFINITY)),
                () -> assertThrows(IllegalArgumentException.class, () -> cruise.update(80.0, Double.NEGATIVE_INFINITY))
        );
    }
}
