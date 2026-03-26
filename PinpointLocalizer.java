package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class PinpointLocalizer extends Localizer{

    GoBildaPinpointDriver odo;
    //double previousHeading; //current heading from 0 to 2pi
    //double totalHeading;
    VectorF pinpointPose;

    PinpointLocalizer(HardwareMap hardwareMap, VectorF _initPose, VectorF _initVel, double _xOff, double _yOff, GoBildaPinpointDriver.GoBildaOdometryPods _podType, GoBildaPinpointDriver.EncoderDirection[] _directionList){

        super(_initPose, _initVel);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(_xOff, _yOff); //118, 126
        odo.setEncoderResolution(_podType); //GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        odo.setEncoderDirections(_directionList[0], _directionList[1]);
        odo.resetPosAndIMU();

    }

    void updateTelemetry(){

        telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
        telemetry.addData("frequency", odo.getFrequency());
        telemetry.addData("x pos", odo.getXOffset());
        telemetry.addData("y pos", odo.getYOffset());
        telemetry.addData("heading", odo.getYawScalar());
        telemetry.update();

    }

    void updatePos(){

        odo.update();
        pinpointPose.put(0, (float) odo.getPosX());
        pinpointPose.put(1, (float) odo.getPosY());
        pinpointPose.put(2, (float) odo.getHeading());

    }

}
