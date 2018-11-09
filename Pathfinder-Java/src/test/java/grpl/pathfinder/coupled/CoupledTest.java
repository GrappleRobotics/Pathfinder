package grpl.pathfinder.coupled;

import grpl.pathfinder.Vec2;
import grpl.pathfinder.path.*;
import grpl.pathfinder.profile.Profile;
import grpl.pathfinder.profile.TrapezoidalProfile;
import grpl.pathfinder.transmission.DcMotor;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

public class CoupledTest {

    DcMotor motor;
    CoupledChassis chassis;
    CoupledDrivetrain drivetrain;

    @BeforeEach
    public void setup() {
        // Dual CIM
        motor = new DcMotor(12.0, 5330*2.0*Math.PI / 60.0 / 12.75, 2 * 2.7, 2*131.0, 2*2.41*12.75);
        chassis = new CoupledChassis(motor, motor, 0.0762, 0.5, 25.0);
        drivetrain = new CoupledDrivetrain(chassis);
    }

    @AfterEach
    public void tearDown() throws Exception {
        motor.close();
        chassis.close();
        drivetrain.close();
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
                state.time(), state.configuration().position().x(), state.configuration().position().y(),
                state.configuration().heading(), state.kinematics().s(), state.kinematics().v(), state.kinematics().a(),
                state.curvature(), 0, 0, 0,
        };
        csv(writer, obj);
    }

    private void echo(Writer writer, CoupledWheelState state, int idx) throws IOException {
        Object obj[] = new Object[]{
                state.time(), state.position().x(), state.position().y(),
                0, state.kinematics().s(), state.kinematics().v(), state.kinematics().a(),
                0, idx, state.voltage(), state.current()
        };
        csv(writer, obj);
    }

    @Test
    public void testCdtHermite() throws IOException {
        TrapezoidalProfile profile = new TrapezoidalProfile();

        List<HermiteWaypoint> waypoints = new ArrayList<>();
        waypoints.add(new HermiteWaypoint(Vec2.cartesian(0, 0), Vec2.cartesian(5, 0), Vec2.cartesian(0, 0)));
        waypoints.add(new HermiteWaypoint(Vec2.cartesian(4, 4), Vec2.cartesian(0, 5), Vec2.cartesian(0, 0)));

        List<HermiteQuintic> hermites = HermiteFactory.generateQuintic(waypoints);
        ArcParameterizer param = new ArcParameterizer();
        param.configure(0.01, 0.01);
        List<? extends Curve2d> curves = param.parameterize(hermites);

        drivetrain.configure(curves, profile);

        CoupledState state = new CoupledState();

        FileWriter out = new FileWriter("cdt.csv");
        csv(out, new String[] {
                "t", "x", "y", "heading", "distance", "velocity",
                "acceleration", "curvature", "path", "voltage", "current"
        });

        for (double t = 0; !state.isFinished() && t < 5; t += 0.01) {
            state = drivetrain.generate(state, t);
//            echo(out, state);

            CoupledWheelState[] wheels = drivetrain.split(state);
//            echo(out, wheels[CoupledDrivetrain.LEFT], 1);
//            echo(out, wheels[CoupledDrivetrain.RIGHT], 2);
        }

        out.close();
        param.close();
        profile.close();

    }
}