package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public abstract class Path {

    public abstract VectorF getPosition(double t);

    public abstract VectorF getVelocity(double t);

    public abstract double getDuration();

}