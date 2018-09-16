package grpl.pathfinder.path;

import java.util.List;
import java.util.Vector;

public class HermiteFactory {

    List<HermiteCubic> generateCubic(List<HermiteWaypoint> wps) {
        if (wps.size() == 0)
            return null;

        int num_curves = wps.size() - 1;
        List<HermiteCubic> list = new Vector<HermiteCubic>(num_curves);

        for (int i = 0; i < num_curves; i++) {
            HermiteWaypoint start = wps.get(i), end = wps.get(i+1);
            list.add(new HermiteCubic(start, end));
        }
        return list;
    }

    List<HermiteQuintic> generateQuintic(List<HermiteWaypoint> wps) {
        if (wps.size() == 0)
            return null;

        int num_curves = wps.size() - 1;
        List<HermiteQuintic> list = new Vector<HermiteQuintic>(num_curves);

        for (int i = 0; i < num_curves; i++) {
            HermiteWaypoint start = wps.get(i), end = wps.get(i+1);
            list.add(new HermiteQuintic(start, end));
        }
        return list;
    }

}
