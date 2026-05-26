package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

public class SegmentedPath {

    ArrayList<Segment> segments;
    int N;
    ArrayList<Double> startTimes;
    ArrayList<Double> endTimes;
    double index = 0;
    SegmentedPath(ArrayList<Segment> _segments){

        segments = _segments;
        N = segments.size();
        for(int i = 0; i < N; i++){
            startTimes.add(segments.get(i).startTime);
            endTimes.add(segments.get(i).endTime);
        }

    }

    public void updateIndex(double t){ //state machine (tells us which segment to be following given a time)

        for(int i = 0; i < N; i++){

            if(t > startTimes.get(i)){
                if(t < endTimes.get(i)){
                    index = i;
                    break;
                }
            }
            else{
                index = i-0.5;
                break;
            }

        }

    }

    public double getx(double t){
        updateIndex(t);
        return 0;
    }

    public double gety(double t){
        return 0;
    }

    public double geth(double t){
        return 0;
    }

    public VectorF getPose(double t){

        return new VectorF((float)getx(t), (float)gety(t), (float)geth(t));

    }

    public double getdx(double t){
        return 0;
    }

    public double getdy(double t){
        return 0;
    }

    public double getdh(double t){
        return 0;
    }

    public VectorF getDerivative(double t){

        return new VectorF((float)getdx(t), (float)getdy(t), (float)getdh(t));

    }

}
