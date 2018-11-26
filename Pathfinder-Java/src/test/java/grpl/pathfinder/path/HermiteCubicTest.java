package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class HermiteCubicTest {

    HermiteCubic.Waypoint start, end;
    HermiteCubic spline;

    @BeforeEach
    void createSpline() {
        start = new HermiteCubic.Waypoint(Vec2.cartesian(0, 0), Vec2.polar(5, 0));
        end = new HermiteCubic.Waypoint(Vec2.cartesian(5, 4), Vec2.polar(5, -Math.PI / 2.0));
        spline = new HermiteCubic(start, end);
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
