package grpl.pathfinder.profile;

import grpl.pathfinder.util.NativeResource;
import grpl.pathfinder.util.NativeResourceTest;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TrapezoidalProfileTest extends NativeResourceTest {

    TrapezoidalProfile profile;

    @BeforeEach
    void createProfile() {
        profile = new TrapezoidalProfile();
    }

    @Override
    protected NativeResource nativeResource() {
        return profile;
    }

    @Test
    void testGoal() {
        profile.setGoal(12.0);
        assertEquals(12.0, profile.getGoal(), 1e-12);
    }

    @Test
    void testTimeslice() {
        profile.setTimeslice(0.123);
        assertEquals(0.123, profile.getTimeslice(), 1e-12);
    }

    @Test
    void testProfile() {
        profile.applyLimit(Profile.VELOCITY, -3, 3);
        profile.applyLimit(Profile.ACCELERATION, -3, 4);
        profile.setGoal(5.0);
        profile.setTimeslice(0);

        Profile.State state = profile.createState();
        for (double t = 0; t < 7; t+=0.01) {
            state = profile.calculate(state, t);
            assertTrue(Math.abs(state.kinematics[Profile.VELOCITY]) <= 3);
            assertTrue(Math.abs(state.kinematics[Profile.VELOCITY]) <= 4);
        }
        assertEquals(5.0, state.kinematics[Profile.POSITION], 0.001);
    }
}
