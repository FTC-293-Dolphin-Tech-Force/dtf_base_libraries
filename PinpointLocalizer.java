package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class PinpointLocalizer extends Localizer{

    GoBildaPinpointDriver odo;
    //double previousHeading; //current heading from 0 to 2pi
    //double totalHeading;
    VectorF pinpointPose = new VectorF(0, 0, 0);
    VectorF pinpointVel = new VectorF(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, ElapsedTime _runtime, VectorF _initPose, VectorF _initVel, double _xOff, double _yOff, GoBildaPinpointDriver.GoBildaOdometryPods _podType, GoBildaPinpointDriver.EncoderDirection[] _directionList){

        super(_initPose, _initVel, _runtime);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(_xOff, _yOff); //118, 126
        odo.setEncoderResolution(_podType); //GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        odo.setEncoderDirections(_directionList[0], _directionList[1]);
        odo.resetPosAndIMU();

    }

    public void updateTelemetry(){

        telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
        telemetry.addData("frequency", odo.getFrequency());
        telemetry.addData("x pos", odo.getXOffset());
        telemetry.addData("y pos", odo.getYOffset());
        telemetry.addData("heading", odo.getYawScalar());
        telemetry.update();

    }

    public void updatePose(){ //m & rad

        odo.update();
        pinpointPose.put(0, (float) (odo.getPosX()*0.001));
        pinpointPose.put(1, (float) (odo.getPosY()*0.001));
        pinpointPose.put(2, (float) odo.getHeading());
        pose = pinpointPose;

    }

    public void updateVel(){ //m s-1 & rad s-1

        odo.update();
        pinpointVel.put(0, (float) (odo.getVelX()*0.001));
        pinpointVel.put(1, (float) (odo.getVelY()*0.001));
        pinpointVel.put(2, (float) odo.getHeadingVelocity());
        vel = pinpointVel;

    }

}
