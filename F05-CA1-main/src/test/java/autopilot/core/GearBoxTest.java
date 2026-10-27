package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.*;

class GearBoxTest {
    private static final double STOPPED_KMH = 0.0;
    private static final double MOVING_KMH = 20.0;

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = {"N", "D"})
    void changeTo_fromParkToForbiddenGear_throwsAndKeepsPark(GearBox.Gear target) {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearbox.changeTo(target));
        assertEquals(GearBox.Gear.P, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromParkToReverseBelowSpeedLimit_succeeds() {
        GearBox gearbox = new GearBox(() -> 4.999);

        gearbox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromParkToReverseAtSpeedLimit_throwsAndKeepsPark() {
        GearBox gearbox = new GearBox(() -> 5.0);

        assertThrows(IllegalGearChangeException.class, () -> gearbox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.P, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromReverseToNeutralWhileMoving_succeeds() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);
        gearbox.setSpeedSensor(() -> MOVING_KMH);

        gearbox.changeTo(GearBox.Gear.N);

        assertEquals(GearBox.Gear.N, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromNeutralToDriveWhileMoving_succeeds() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);
        gearbox.changeTo(GearBox.Gear.N);
        gearbox.setSpeedSensor(() -> MOVING_KMH);

        gearbox.changeTo(GearBox.Gear.D);

        assertEquals(GearBox.Gear.D, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromReverseToDriveWhileStopped_succeeds() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);

        gearbox.changeTo(GearBox.Gear.D);

        assertEquals(GearBox.Gear.D, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromReverseToDriveWhileMoving_throwsAndKeepsReverse() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);
        gearbox.setSpeedSensor(() -> MOVING_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearbox.changeTo(GearBox.Gear.D));
        assertEquals(GearBox.Gear.R, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromDriveToReverseWhileStopped_succeeds() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);
        gearbox.changeTo(GearBox.Gear.D);

        gearbox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_fromDriveToReverseWhileMoving_throwsAndKeepsDrive() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        gearbox.changeTo(GearBox.Gear.R);
        gearbox.changeTo(GearBox.Gear.D);
        gearbox.setSpeedSensor(() -> MOVING_KMH);

        assertThrows(IllegalGearChangeException.class, () -> gearbox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.D, gearbox.getCurrentGear());
    }

    @ParameterizedTest
    @CsvSource({"R, P", "N, P", "N, R", "D, P", "D, N"})
    void changeTo_betweenForbiddenGears_throwsAndKeepsCurrentGear(
            GearBox.Gear initial, GearBox.Gear target) {
        GearBox gearbox = gearboxIn(initial);

        assertThrows(IllegalGearChangeException.class, () -> gearbox.changeTo(target));
        assertEquals(initial, gearbox.getCurrentGear());
    }

    @Test
    void changeTo_toSameGear_doesNotReadSpeedSensor() {
        GearBox gearbox = new GearBox(() -> {
            throw new AssertionError("Speed sensor should not be read for a no-op shift");
        });

        assertDoesNotThrow(() -> gearbox.changeTo(GearBox.Gear.P));
    }

    @Test
    void changeTo_withNullTarget_throwsNullPointerException() {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);

        assertThrows(NullPointerException.class, () -> gearbox.changeTo(null));
    }

    @Test
    void changeTo_withNegativeSpeed_throwsIllegalArgumentException() {
        GearBox gearbox = new GearBox(() -> -1.0);

        assertThrows(IllegalArgumentException.class, () -> gearbox.changeTo(GearBox.Gear.R));
    }

    @Test
    void changeTo_withNonFiniteSpeed_throwsIllegalArgumentException() {
        GearBox gearbox = new GearBox(() -> Double.NaN);
        assertThrows(IllegalArgumentException.class, () -> gearbox.changeTo(GearBox.Gear.R));

        gearbox.setSpeedSensor(() -> Double.POSITIVE_INFINITY);
        assertThrows(IllegalArgumentException.class, () -> gearbox.changeTo(GearBox.Gear.R));

        gearbox.setSpeedSensor(() -> Double.NEGATIVE_INFINITY);
        assertThrows(IllegalArgumentException.class, () -> gearbox.changeTo(GearBox.Gear.R));
    }

    private static GearBox gearboxIn(GearBox.Gear desiredGear) {
        GearBox gearbox = new GearBox(() -> STOPPED_KMH);
        if (desiredGear != GearBox.Gear.P) gearbox.changeTo(GearBox.Gear.R);
        if (desiredGear == GearBox.Gear.N || desiredGear == GearBox.Gear.D) {
            gearbox.changeTo(GearBox.Gear.N);
        }
        if (desiredGear == GearBox.Gear.D) gearbox.changeTo(GearBox.Gear.D);
        return gearbox;
    }

    @Test
    void changeTo_whenSpeedSensorFails_propagatesException() {
        GearBox gearbox = new GearBox(() -> {
            throw new SensorFailureException("speed sensor unavailable");
        });

        assertThrows(SensorFailureException.class, () -> gearbox.changeTo(GearBox.Gear.R));
    }
}
