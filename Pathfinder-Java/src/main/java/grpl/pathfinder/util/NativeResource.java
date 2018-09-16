package grpl.pathfinder.util;

public abstract class NativeResource extends PathfinderJNI implements AutoCloseable {
    public abstract boolean closed();
}
