package grpl.pathfinder.path;

public interface Curve2d {

    Vec2 position(double s);
    Vec2 velocity(double s);

    Vec2 rotation(double s);

    double curvature(double s);
    double curvature_prime(double s);

    double length();

}
