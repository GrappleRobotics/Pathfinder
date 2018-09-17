package grpl.pathfinder.path;

public interface Curve2d {

    Vec2 position(double t);
    Vec2 velocity(double t);

    Vec2 rotation(double t);

    double curvature(double t);
    double curvature_prime(double t);

    double length();

}
