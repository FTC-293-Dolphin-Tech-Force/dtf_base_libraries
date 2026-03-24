package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class PinpointLocalizer extends Localizer{

    GoBildaPinpointDriver odo;
    double previousHeading;
    double totalHeading;
    VectorF pinpointPose;

    PinpointLocalizer(HardwareMap hardwareMap, VectorF _initPose, VectorF _initVel){

        super(_initPose, _initVel);

    }

}
