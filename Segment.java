package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Segment extends Path {

    private final Path path;
    private final double duration;


    public Segment(Path _path, double _duration) {

        if (_path == null) {throw new IllegalArgumentException("Null segment detected");}

        path = _path;
        duration = Math.abs(_duration);

    }


    @Override
    public VectorF getPosition(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));

        return path.getPosition(t);

    }


    @Override
    public VectorF getVelocity(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));

        return path.getVelocity(t);
    }


    @Override
    public double getDuration() {
        return duration;
    }
}