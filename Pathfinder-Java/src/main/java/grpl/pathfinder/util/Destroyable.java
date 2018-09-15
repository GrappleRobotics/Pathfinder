package grpl.pathfinder.util;

public abstract class Destroyable extends PathfinderJNI {
    public abstract void destroy();
    public abstract boolean destroyed();

    @Override
    protected void finalize() {
        if (!destroyed()) {
            System.err.println("WARNING: This object has reached the end of its life, but you haven't called destroy()! " +
                    "This is a possible memory leak. Object: " + this.toString());
            destroy();
        }
    }
}
