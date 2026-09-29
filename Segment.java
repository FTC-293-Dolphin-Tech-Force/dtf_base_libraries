package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public abstract class Segment {

    protected double duration;

    protected VectorF origin = new VectorF(0, 0); //the 2d origin of the reference frame. This is added to the output of getPosition.

    public Segment(double _duration) {
        duration = Math.abs(_duration);
    }

    public abstract VectorF getPosition(double t);

    public abstract VectorF getVelocity(double t);

    public double getDuration() {
        return duration;
    }

    public void setOrigin(VectorF newOrigin){
        origin = newOrigin;
    }

    public void shiftOrigin(VectorF shift){
        origin.add(shift);
    }

    public VectorF getOrigin(){
        return origin;
    }

}