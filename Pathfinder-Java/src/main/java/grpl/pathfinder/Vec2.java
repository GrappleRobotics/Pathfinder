package grpl.pathfinder;

public class Vec2 {

    private final double x, y;

    private Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double[] xy() {
        return new double[] { x, y };
    }

    public Vec2 unit() {
        double mag = magnitude();
        return Vec2.cartesian(x / mag, y / mag);
    }

    public Vec2 plus(Vec2 other) {
        return new Vec2(x() + other.x(), y() + other.y());
    }

    public Vec2 minus(Vec2 other) {
        return new Vec2(x() - other.x(), y() - other.y());
    }

    public Vec2 times(double scalar) {
        return new Vec2(x() * scalar, y() * scalar);
    }

    public static Vec2 cartesian(double x, double y) {
        return new Vec2(x, y);
    }

    public static Vec2 cartesian(double[] xy) {
        return cartesian(xy[0], xy[1]);
    }

    public static Vec2 polar(double mag, double theta) {
        return new Vec2(mag * Math.cos(theta), mag * Math.sin(theta));
    }

    public static Vec2 zero() {
        return new Vec2(0, 0);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vec2))
            return false;
        Vec2 vec2 = (Vec2) o;
        return Math.abs(vec2.x() - x()) < 1e-6 &&
                Math.abs(vec2.y() - y()) < 1e-6;
    }
}
