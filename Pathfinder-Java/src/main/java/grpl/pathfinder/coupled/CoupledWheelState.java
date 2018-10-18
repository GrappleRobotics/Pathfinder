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

    public CoupledWheelState(double[] arr) {
        this.time = arr[0];

        double[] posArr = new double[] { arr[1], arr[2] };
        double[] kinArr = new double[] { arr[3], arr[4], arr[5] };

        this.position = Vec2.cartesian(posArr);
        this.kinematics = new CoupledKinematics(kinArr);

        this.voltage = arr[6];
        this.current = arr[7];
        this.finished = arr[8] > 0.5;
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

    public double[] toArray() {
        double[] posArr = position.xy();
        double[] kinArr = kinematics.toArray();

        double[] arr = new double[4 + kinArr.length + posArr.length];
        arr[0] = time;
        for (int i = 0; i < posArr.length; i++)
            arr[1+i] = posArr[i];
        for (int i = 0; i < kinArr.length; i++)
            arr[3+i] = kinArr[i];

        arr[6] = voltage;
        arr[7] = current;
        arr[8] = finished ? 1.0 : 0.0;
        return arr;
    }

}
