package grpl.pathfinder.path;

public interface Spline2d {

    Vec2 position(double t);
    Vec2 velocity(double t);

    Vec2 rotation(double t);

    double curvature(double t);

}
