package grpl.pathfinder.coupled;

/**
 * The state of a coupled drivetrain at any point in time, as a single state within a
 * trajectory.
 */
public class CoupledState {

    /**
     * The time point of this state, in seconds.
     */
    public double time;

    /**
     * The instantaneous curvature of the state, in m^-1
     */
    public double curvature;

    /**
     * The instantaneous change in curvature of the state, in m^-2 (dk/ds)
     */
    public double dcurvature;

    /**
     * The configuration of the chassis at the time of the state.
     */
    public CoupledConfigurationState config;

    /**
     * The kinematics of the chassis at the time of the state.
     */
    public CoupledKinematicState kinematics;

    public boolean finished;

    public CoupledState() {
        this.time = 0;
        this.curvature = 0;
        this.dcurvature = 0;
        this.config = new CoupledConfigurationState();
        this.kinematics = new CoupledKinematicState();
        this.finished = false;
    }

    public CoupledState(double time, double curvature, double dcurvature,
                        CoupledConfigurationState config, CoupledKinematicState kin, boolean finished) {
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

        double[] con = new double[]{arr[3], arr[4], arr[5]};
        double[] kin = new double[]{arr[6], arr[7], arr[8]};

        this.finished = arr[9] > 0.5;

        this.kinematics = new CoupledKinematicState(kin);
        this.config = new CoupledConfigurationState(con);
    }

    public double[] toArray() {
        double[] kinArr = kinematics.toArray();
        double[] conArr = config.toArray();

        double[] arr = new double[4 + kinArr.length + conArr.length];
        arr[0] = time;
        arr[1] = curvature;
        arr[2] = dcurvature;
        for (int i = 0; i < conArr.length; i++)
            arr[3 + i] = conArr[i];
        for (int i = 0; i < kinArr.length; i++)
            arr[6 + i] = kinArr[i];
        arr[9] = finished ? 1.0 : 0.0;
        return arr;
    }

}
