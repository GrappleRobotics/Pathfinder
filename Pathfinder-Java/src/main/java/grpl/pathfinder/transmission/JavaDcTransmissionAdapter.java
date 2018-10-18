package grpl.pathfinder.transmission;

import grpl.pathfinder.util.NativeResource;

public class JavaDcTransmissionAdapter extends AbstractDcTransmission {

    public  JavaDcTransmissionAdapter(DcTransmission transmission) {
        super(allocate(transmission));
    }

    private static native long allocate(DcTransmission javaTransmission);

}
