package grpl.pathfinder.coupled;

import grpl.pathfinder.Vec2;
import grpl.pathfinder.path.ArcParameterizer;
import grpl.pathfinder.path.Curve2d;
import grpl.pathfinder.path.HermiteFactory;
import grpl.pathfinder.path.HermiteQuintic;
import grpl.pathfinder.profile.TrapezoidalProfile;
import grpl.pathfinder.transmission.DcMotor;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

public class CoupledTest {

    DcMotor motor;
    CoupledChassis chassis;
    CoupledCausalTrajGen gen;

    @BeforeEach
    public void setup() {
        // Dual CIM
        motor = new DcMotor(12.0, 5330 * 2.0 * Math.PI / 60.0 / 12.75, 2 * 2.7, 2 * 131.0, 2 * 2.41 * 12.75);
        chassis = new CoupledChassis(motor, motor, 0.0762, 0.5, 25.0);
        gen = new CoupledCausalTrajGen(chassis);
    }

    @AfterEach
    public void tearDown() throws Exception {
        motor.close();
        chassis.close();
        gen.close();
    }

    private void csv(Writer writer, Object[] obj) throws IOException {
        for (int i = 0; i < obj.length; i++) {
            writer.write(obj[i].toString());
            if (i != obj.length - 1)
                writer.write(",");
        }
        writer.write("\n");
    }

    private void echo(Writer writer, CoupledState state) throws IOException {
        Object obj[] = new Object[]{
                state.time, state.config.position.x(), state.config.position.y(),
                state.config.heading, state.kinematics.distance, state.kinematics.velocity, state.kinematics.acceleration,
                state.curvature, 0, 0, 0,
        };
        csv(writer, obj);
    }

    private void echo(Writer writer, CoupledWheelState state, int idx) throws IOException {
        Object obj[] = new Object[]{
                state.time, state.position.x(), state.position.y(),
                0, state.kinematics.distance, state.kinematics.velocity, state.kinematics.acceleration,
                0, idx, state.voltage, state.current
        };
        csv(writer, obj);
    }

    @Test
    public void testCdtHermite() throws IOException {
        TrapezoidalProfile profile = new TrapezoidalProfile();

        List<HermiteQuintic.Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new HermiteQuintic.Waypoint(Vec2.cartesian(0, 0), Vec2.cartesian(5, 0), Vec2.cartesian(0, 0)));
        waypoints.add(new HermiteQuintic.Waypoint(Vec2.cartesian(4, 4), Vec2.cartesian(0, 5), Vec2.cartesian(0, 0)));

        List<HermiteQuintic> hermites = HermiteFactory.generateQuintic(waypoints);
        ArcParameterizer param = new ArcParameterizer();
        param.configure(0.01, 0.01);
        List<? extends Curve2d> curves = param.parameterize(hermites);

        gen.configure(curves, profile);

        CoupledState state = new CoupledState();

//        FileWriter out = new FileWriter("cdt.csv");
//        csv(out, new String[] {
//                "t", "x", "y", "heading", "distance", "derivative",
//                "acceleration", "curvature", "path", "voltage", "current"
//        });

        for (double t = 0; !state.finished && t < 5; t += 0.01) {
            state = gen.generate(state, t);
//            echo(out, state);

            CoupledWheelState[] wheels = chassis.split(state);
//            echo(out, wheels[CoupledCausalTrajGen.LEFT], 1);
//            echo(out, wheels[CoupledCausalTrajGen.RIGHT], 2);
        }

//        out.close();
        param.close();
        profile.close();

    }
}
