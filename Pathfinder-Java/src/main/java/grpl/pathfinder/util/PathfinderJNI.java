package grpl.pathfinder.util;

public class PathfinderJNI {

    static boolean _loaded = false;

    static {
        loadPathfinderJNI();
    }

    public static boolean loadPathfinderJNI() {
        if (!_loaded) {
            System.loadLibrary("pathfinderjni");
            _loaded = true;
            return true;
        }
        return false;
    }

}
