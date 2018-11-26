package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ArcParameterizerTest {

    ArcParameterizer arcParam;
    List<? extends Spline2d> nativeSplines;

    @BeforeEach
    void createParam() {
        arcParam = new ArcParameterizer();
        arcParam.configure(0.1, 0.1);

        List<HermiteCubic.Waypoint> wps = new ArrayList<>();
        wps.add(new HermiteCubic.Waypoint(Vec2.cartesian(2, 2), Vec2.cartesian(5, 0)));
        wps.add(new HermiteCubic.Waypoint(Vec2.cartesian(5, 5), Vec2.cartesian(3, 2)));
        wps.add(new HermiteCubic.Waypoint(Vec2.cartesian(10, 5), Vec2.cartesian(0, 5)));
        nativeSplines = HermiteFactory.generateCubic(wps);
    }

    @Test
    void testParameterization() throws IOException {
        List<Arc2d> arcs = arcParam.parameterize(nativeSplines);
//        BufferedWriter writer = new BufferedWriter(new FileWriter("javaparam.csv"));
//        writer.write("x,y\n");
//        arcs.forEach((arc) -> {
//            for (double s = 0; s < arc.length(); s+=0.01) {
//                Vec2 pos = arc.position(s);
//                try {
//                    writer.write(pos.x() + "," + pos.y() + "\n");
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }
//        });
//        writer.close();
    }

}
