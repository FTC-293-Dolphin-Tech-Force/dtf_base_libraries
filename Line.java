package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Line extends Spline{

    public Line(VectorF _startPose, VectorF _endPose, double _duration){
        super(  new double[] {_startPose.get(0), (_endPose.get(0)-_startPose.get(0))/_duration},
                new double[] {_startPose.get(1), (_endPose.get(1)-_startPose.get(1))/_duration},
                new double[] {_startPose.get(2), (_endPose.get(2)-_startPose.get(2))/_duration},
                _duration);
    }

}
