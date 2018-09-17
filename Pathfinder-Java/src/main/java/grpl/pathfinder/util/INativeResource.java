package grpl.pathfinder.util;

public interface INativeResource extends AutoCloseable {
    long nativeHandle();
    boolean closed();
}
