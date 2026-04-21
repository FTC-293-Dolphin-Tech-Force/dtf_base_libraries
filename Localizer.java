package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Localizer {

    VectorF pose; //m & rad
    VectorF lastPose;
    VectorF vel;


    ElapsedTime runtime;
    double lastTime = 0;

    Localizer(VectorF _initPose, VectorF _initVel, ElapsedTime _runtime){

        setPose(_initPose);
        setVel(_initVel);
        runtime = _runtime;

    }

    public VectorF measureVel(){

        double dt = runtime.seconds() - lastTime; // final minus initial
        lastTime = runtime.seconds();

        VectorF dr = pose.subtracted(lastPose); // final minus initial
        VectorF measuredVel = dr.multiplied((float)(1.0/dt));

        return measuredVel;

    }

    public VectorF getPoseError(VectorF expectedPose){

        updatePose();
        VectorF error = pose.subtracted(expectedPose);
        float ht = expectedPose.get(2); //h target
        float h = pose.get(2); //maybe just use getHeading for this and replace the first line
        float hError = (float)(((h-Math.PI)%(2*Math.PI))+(Math.PI-ht));
        error.put(2, hError);
        return error;

    }

    void setPose(VectorF _pose){

        pose = _pose;

    }

    void setPose(float x, float y, float heading){

        pose = new VectorF(x, y, heading);
        lastPose = pose;

    }
    void setVel(VectorF _vel){

        vel = _vel;

    }

    public VectorF getPose() {
        updatePose();
        return pose;
    }

    public float getHeading() {
        updatePose();
        return pose.get(2);
    }

    public VectorF getVel() {
        updateVel();
        return vel;
    }

    public void updatePose(){}

    public void updateVel(){}

    public void updateTelemetry(){

        telemetry.addData("Localizer Status", "Working");
        telemetry.update();

    }

}
