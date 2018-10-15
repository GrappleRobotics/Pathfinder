package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

public interface Spline2d {

    Vec2 position(double t);
    Vec2 velocity(double t);

    Vec2 rotation(double t);

    double curvature(double t);

}
