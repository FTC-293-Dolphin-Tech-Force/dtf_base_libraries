package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Segment extends Path{

    Path path;
    double startTime;
    double endTime;
    VectorF startPose;
    VectorF startDerivative;
    VectorF endPose;
    VectorF endDerivative;
    Segment(Path _path, double _startTime, double _endTime, VectorF _startPose, VectorF _startDerivative, VectorF _endPose, VectorF _endDerivative) {

        path = _path;
        startTime = _startTime;
        endTime = _endTime;
        startPose = _startPose;
        startDerivative = _startDerivative;
        endPose = _endPose;
        endDerivative = _endDerivative;

    }
    Segment(Path _path, double _startTime, double _endTime) {

        path = _path;
        startTime = _startTime;
        endTime = _endTime;
        startPose = path.getPose(startTime);
        startDerivative = path.getDerivative(startTime);
        startPose = path.getPose(endTime);
        startDerivative = path.getDerivative(endTime);

    }

    Segment(Path _path, double _endTime) {

        path = _path;
        startTime = 0;
        endTime = _endTime;
        startPose = path.getPose(startTime);
        startDerivative = path.getDerivative(startTime);
        startPose = path.getPose(endTime);
        startDerivative = path.getDerivative(endTime);

    }

    public double getx(double t){
        if(t < startTime){
            return startPose.get(0);
        } else if (t > endTime) {
            return endPose.get(0);
        }
        else{
            return path.getx(t);
        }
    }

    public double gety(double t){
        if(t < startTime){
            return startPose.get(1);
        } else if (t > endTime) {
            return endPose.get(1);
        }
        else{
            return path.gety(t);
        }
    }

    public double geth(double t){
        if(t < startTime){
            return startPose.get(2);
        } else if (t > endTime) {
            return endPose.get(2);
        }
        else{
            return path.geth(t);
        }
    }

    public double getdx(double t){
        if(t < startTime){
            return startDerivative.get(0);
        } else if (t > endTime) {
            return endDerivative.get(0);
        }
        else{
            return path.getdx(t);
        }
    }

    public double getdy(double t){
        if(t < startTime){
            return startDerivative.get(1);
        } else if (t > endTime) {
            return endDerivative.get(1);
        }
        else{
            return path.getdy(t);
        }
    }

    public double getdh(double t){
        if(t < startTime){
            return startDerivative.get(2);
        } else if (t > endTime) {
            return endDerivative.get(2);
        }
        else{
            return path.getdh(t);
        }
    }

}
