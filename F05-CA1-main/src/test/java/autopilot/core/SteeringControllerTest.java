package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class SteeringControllerTest {
    private static final double EPSILON = 1e-9;
    private static final double NORMAL_SPEED_KMH = 60.0;

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_nonFiniteRequestedAngle_throwsIllegalArgumentException(double requestedAngle) {
        SteeringController steering = new SteeringController();

        assertThrows(IllegalArgumentException.class,
                () -> steering.update(requestedAngle, NORMAL_SPEED_KMH));
    }

    @Test
    void update_withinOneTickReachesRequestedAngle() {
        SteeringController steering = new SteeringController();

        assertEquals(45.0, steering.update(45.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_atSixtyDegreeChange_reachesTargetInOneTick() {
        SteeringController steering = new SteeringController();

        assertEquals(60.0, steering.update(60.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_aboveMaximumChange_movesAtMostSixtyDegrees() {
        SteeringController steering = new SteeringController();

        assertEquals(60.0, steering.update(120.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_withRepeatedTicks_reachesRequestedAngleGradually() {
        SteeringController steering = new SteeringController();
        steering.update(150.0, NORMAL_SPEED_KMH);
        steering.update(150.0, NORMAL_SPEED_KMH);

        assertEquals(150.0, steering.update(150.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_whenChangingDirection_respectsPerTickLimit() {
        SteeringController steering = new SteeringController();
        steering.update(60.0, NORMAL_SPEED_KMH);

        assertEquals(0.0, steering.update(-60.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_negativeRequestedAngle_movesInNegativeDirection() {
        SteeringController steering = new SteeringController();

        assertEquals(-60.0, steering.update(-100.0, NORMAL_SPEED_KMH), EPSILON);
    }

    @Test
    void update_abovePhysicalLimit_clampsTargetToPositive450Degrees() {
        SteeringController steering = new SteeringController();

        steering.update(500.0, NORMAL_SPEED_KMH);

        assertEquals(450.0, steering.getTargetAngleDegrees(), EPSILON);
    }

    @Test
    void update_belowPhysicalLimit_clampsTargetToNegative450Degrees() {
        SteeringController steering = new SteeringController();

        steering.update(-500.0, NORMAL_SPEED_KMH);

        assertEquals(-450.0, steering.getTargetAngleDegrees(), EPSILON);
    }

    @Test
    void update_atHighSpeedThreshold_usesNormalSteeringLimit() {
        SteeringController steering = new SteeringController();

        steering.update(200.0, 120.0);

        assertEquals(200.0, steering.getTargetAngleDegrees(), EPSILON);
    }

    @Test
    void update_aboveHighSpeedThreshold_limitsPositiveTargetTo90Degrees() {
        SteeringController steering = new SteeringController();

        steering.update(200.0, 120.1);

        assertEquals(90.0, steering.getTargetAngleDegrees(), EPSILON);
    }

    @Test
    void update_aboveHighSpeedThreshold_limitsNegativeTargetToMinus90Degrees() {
        SteeringController steering = new SteeringController();

        steering.update(-200.0, 120.1);

        assertEquals(-90.0, steering.getTargetAngleDegrees(), EPSILON);
    }

    @Test
    void update_withNegativeSpeed_throwsIllegalArgumentException() {
        SteeringController steering = new SteeringController();

        assertThrows(IllegalArgumentException.class, () -> steering.update(10.0, -1.0));
    }

    @Test
    void update_withNonFiniteSpeed_throwsIllegalArgumentException() {
        SteeringController steering = new SteeringController();

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> steering.update(10.0, Double.NaN)),
                () -> assertThrows(IllegalArgumentException.class, () -> steering.update(10.0, Double.POSITIVE_INFINITY)),
                () -> assertThrows(IllegalArgumentException.class, () -> steering.update(10.0, Double.NEGATIVE_INFINITY))
        );
    }

    @Test
    void selfCenter_fromSmallAngle_returnsDirectlyToZero() {
        SteeringController steering = new SteeringController();
        steering.update(45.0, NORMAL_SPEED_KMH);

        assertEquals(0.0, steering.selfCenter(), EPSILON);
    }

    @Test
    void selfCenter_fromLargeAngle_movesTowardZeroBySixtyDegrees() {
        SteeringController steering = new SteeringController();
        steering.update(180.0, NORMAL_SPEED_KMH);
        steering.update(180.0, NORMAL_SPEED_KMH);
        steering.update(180.0, NORMAL_SPEED_KMH);

        assertEquals(120.0, steering.selfCenter(), EPSILON);
    }

    @Test
    void selfCenter_fromNegativeAngle_movesTowardZero() {
        SteeringController steering = new SteeringController();
        steering.update(-100.0, NORMAL_SPEED_KMH);
        steering.update(-100.0, NORMAL_SPEED_KMH);

        assertEquals(-40.0, steering.selfCenter(), EPSILON);
    }

    @Test
    void selfCenter_whenAlreadyCentered_remainsZero() {
        SteeringController steering = new SteeringController();

        assertEquals(0.0, steering.selfCenter(), EPSILON);
    }
}
