package grpl.pathfinder.coupled;

import grpl.pathfinder.Vec2;

public class CoupledWheelState {

    private double time;
    private Vec2 position;
    private CoupledKinematics kinematics;
    private double voltage, current;
    private boolean finished;

    public CoupledWheelState(double time, Vec2 pos, CoupledKinematics kinem,
                             double voltage, double current, boolean finished) {
        this.time = time;
        this.position = pos;
        this.kinematics = kinem;
        this.voltage = voltage;
        this.current = current;
        this.finished = finished;
    }

    public boolean isFinished() {
        return finished;
    }

    public CoupledKinematics kinematics() {
        return kinematics;
    }

    public Vec2 position() {
        return position;
    }

    public double voltage() {
        return voltage;
    }

    public double current() {
        return current;
    }

    public double time() {
        return time;
    }

}
