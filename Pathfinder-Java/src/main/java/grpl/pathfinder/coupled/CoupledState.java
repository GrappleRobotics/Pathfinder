package grpl.pathfinder.coupled;

public class CoupledState {

    private double time;
    private double curvature, dcurvature;
    private CoupledConfiguration config;
    private CoupledKinematics kinematics;
    private boolean finished;

    public CoupledState() {
        this.time = 0;
        this.curvature = 0;
        this.dcurvature = 0;
        this.config = new CoupledConfiguration();
        this.kinematics = new CoupledKinematics();
        this.finished = false;
    }

    public CoupledState(double time, double curvature, double dcurvature,
                        CoupledConfiguration config, CoupledKinematics kin, boolean finished) {
        this.time = time;
        this.curvature = curvature;
        this.dcurvature = dcurvature;
        this.config = config;
        this.kinematics = kin;
        this.finished = finished;
    }

    public CoupledState(double[] arr) {
        this.time = arr[0];
        this.curvature = arr[1];
        this.dcurvature = arr[2];

        double[] con = new double[] { arr[3], arr[4], arr[5] };
        double[] kin = new double[] { arr[6], arr[7], arr[8] };

        this.finished = arr[9] > 0.5;

        this.kinematics = new CoupledKinematics(kin);
        this.config = new CoupledConfiguration(con);
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

    public double[] toArray() {
        double[] kinArr = kinematics.toArray();
        double[] conArr = config.toArray();

        double[] arr = new double[4 + kinArr.length + conArr.length];
        arr[0] = time;
        arr[1] = curvature;
        arr[2] = dcurvature;
        for (int i = 0; i < conArr.length; i++)
            arr[3+i] = conArr[i];
        for (int i = 0; i < kinArr.length; i++)
            arr[6+i] = kinArr[i];
        arr[9] = finished ? 1.0 : 0.0;
        return arr;
    }

}
