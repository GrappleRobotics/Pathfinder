package grpl.pathfinder.profile;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TrapezoidalProfileTest {

    TrapezoidalProfile profile;

    @BeforeEach
    void createProfile() {
        profile = new TrapezoidalProfile();
    }

    @Test
    void testDestroy() {
        assertFalse(profile.closed());
        profile.close();
        assertTrue(profile.closed());
    }

    @Test
    void testGoal() {
        profile.setGoal(12.0);
        assertEquals(12.0, profile.getGoal(), 1e-12);
    }

    @Test
    void setTimeslice() {
        profile.setTimeslice(0.123);
        assertEquals(0.123, profile.getTimeslice(), 1e-12);
    }

    @Test
    void testProfile() {
        profile.applyLimit(Profile.VELOCITY, -3, 3);
        profile.applyLimit(Profile.ACCELERATION, -3, 4);
        profile.setGoal(5.0);
        profile.setTimeslice(0);

        Profile.Segment segment = profile.createSegment();
        for (double t = 0; t < 7; t+=0.01) {
            segment = profile.calculate(segment, t);
            assertTrue(Math.abs(segment.kinematics[Profile.VELOCITY]) <= 3);
            assertTrue(Math.abs(segment.kinematics[Profile.VELOCITY]) <= 4);
        }
        assertEquals(5.0, segment.kinematics[Profile.POSITION], 0.001);
    }

}
