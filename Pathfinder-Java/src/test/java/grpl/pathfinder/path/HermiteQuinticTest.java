package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class HermiteQuinticTest {

    HermiteQuintic.Waypoint start, end;
    HermiteQuintic spline;

    @BeforeEach
    void createSpline() {
        start = new HermiteQuintic.Waypoint(Vec2.cartesian(0, 0), Vec2.polar(5, 0), Vec2.polar(3,3 ));
        end = new HermiteQuintic.Waypoint(Vec2.cartesian(5, 4), Vec2.polar(5, -Math.PI / 2.0), Vec2.polar(-2, 3));
        spline = new HermiteQuintic(start, end);
    }

    @Test
    void testStartPoint() {
        assertEquals(start.position, spline.position(0));
        assertEquals(start.tangent, spline.derivative(0));
    }

    @Test
    void testEndPoint() {
        assertEquals(end.position, spline.position(1));
        assertEquals(end.tangent, spline.derivative(1));
    }

}
