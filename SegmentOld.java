package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@Deprecated
public class SegmentOld extends PathOld {

    PathOld path;
    double startTime = 0;
    double endTime = 0;
    double duration = 0;
    VectorF startPose = new VectorF(0, 0, 0);
    VectorF startDerivative = new VectorF(0, 0, 0);
    VectorF endPose = new VectorF(0, 0, 0);
    VectorF endDerivative = new VectorF(0, 0, 0);

    @Deprecated
    SegmentOld(PathOld _path, double _startTime, double _endTime, VectorF _startPose, VectorF _startDerivative, VectorF _endPose, VectorF _endDerivative) {

        path = _path;
        startTime = _startTime;
        endTime = _endTime;
        startPose = _startPose;
        startDerivative = _startDerivative;
        endPose = _endPose;
        endDerivative = _endDerivative;

    }

    @Deprecated
    public SegmentOld(PathOld _path, double _startTime, double _endTime) {

        path = _path;
        startTime = _startTime;
        endTime = _endTime;
        startPose = path.getPose(startTime);
        startDerivative = path.getDerivative(startTime);
        startPose = path.getPose(endTime);
        startDerivative = path.getDerivative(endTime);

    }


    SegmentOld(PathOld _path, double _duration) {

        path = _path;
        duration = _duration;

    }

    public double getDuration(){
        return duration;
    }

    public double getx(double t){
        if(t < 0){
            return path.getx(0);
        } else if (t > duration) {
            return path.getx(duration);
        }
        else{
            return path.getx(t);
        }
    }

    public double gety(double t){
        if(t < 0){
            return path.gety(0);
        } else if (t > duration) {
            return path.gety(duration);
        }
        else{
            return path.gety(t);
        }
    }

    public double geth(double t){
        if(t < 0){
            return path.geth(0);
        } else if (t > duration) {
            return path.geth(duration);
        }
        else{
            return path.geth(t);
        }
    }

    public double getdx(double t){
        if(t < 0){
            return path.getdx(0);
        } else if (t > duration) {
            return path.getdx(duration);
        }
        else{
            return path.getdx(t);
        }
    }

    public double getdy(double t){
        if(t < 0){
            return path.getdy(0);
        } else if (t > duration) {
            return path.getdy(duration);
        }
        else{
            return path.getdy(t);
        }
    }

    public double getdh(double t){
        if(t < 0){
            return path.getdh(0);
        } else if (t > duration) {
            return path.getdh(duration);
        }
        else{
            return path.getdh(t);
        }
    }

}
