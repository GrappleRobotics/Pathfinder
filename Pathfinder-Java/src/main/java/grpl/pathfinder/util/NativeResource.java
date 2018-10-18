package grpl.pathfinder.util;

public abstract class NativeResource extends PathfinderJNI implements INativeResource {
    private long _handle = 0;

    public NativeResource(long handle) {
        this._handle = handle;
    }

    @Override
    public long nativeHandle() {
        return _handle;
    }

    protected void zeroHandle() {
        _handle = 0;
    }

    @Override
    public void close() { }

    @Override
    public boolean closed() {
        return _handle == 0;
    }
}
