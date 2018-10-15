package grpl.pathfinder.coupled;

public class CoupledState {

    private double time;
    private double curvature, dcurvature;
    private CoupledConfiguration config;
    private CoupledKinematics kinematics;
    private boolean finished;

    public CoupledState(double time, double curvature, double dcurvature,
                        CoupledConfiguration config, CoupledKinematics kinem, boolean finished) {
        this.time = time;
        this.curvature = curvature;
        this.dcurvature = dcurvature;
        this.config = config;
        this.kinematics = kinem;
        this.finished = finished;
    }

    public boolean isFinished() {
        return finished;
    }

    public CoupledKinematics kinematics() {
        return kinematics;
    }

    public CoupledConfiguration configuration() {
        return config;
    }

    public double curvature() {
        return curvature;
    }

    public double dcurvature() {
        return dcurvature;
    }

    public double time() {
        return time;
    }

}
