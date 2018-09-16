package grpl.pathfinder.util;

public abstract class NativeResource extends PathfinderJNI implements AutoCloseable {
    private long _handle = 0;

    public NativeResource(long handle) {
        this._handle = handle;
    }

    public long nativeHandle() {
        return _handle;
    }

    protected void zeroHandle() {
        _handle = 0;
    }

    public boolean closed() {
        return _handle == 0;
    }
}
