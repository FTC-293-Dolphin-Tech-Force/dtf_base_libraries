package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Localizer {

    VectorF pose;
    VectorF vel;

    Localizer(VectorF _initPose, VectorF _initVel){

        setPose(_initPose);
        setVel(_initVel);

    }

    void setPose(VectorF _pose){

        pose = _pose;

    }

    void setPose(float x, float y, float heading){

        pose = new VectorF(x, y, heading);

    }
    void setVel(VectorF _vel){

        vel = _vel;

    }

    public VectorF getPose() {
        return pose;
    }

    public VectorF getVel() {
        return vel;
    }

    public void updatePos(){}

    public void updateTelemetry(){

        telemetry.addData("Localizer Status", "Working");
        telemetry.update();

    }

}
