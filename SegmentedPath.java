package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

public class SegmentedPath extends Path{

    ArrayList<Segment> segments;
    int N;
    ArrayList<Double> startTimes = new ArrayList<Double>();
    ArrayList<Double> endTimes = new ArrayList<Double>();
    int index = 0; //if index = 2k, we are on segment k. If index = 2k+1, we are between segments k and k+1. if index = -1, we are waiting to start segment 0.
    public SegmentedPath(ArrayList<Segment> _segments){

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
                    index = 2*i; //maybe can be made more efficient by using index += 2, since index never decreases.
                    break;
                }
            }
            else{
                index = 2*i-1;
                break;
            }

        }

    }

    public double getx(double t){ //we might be able to make this more efficient by combining it all into getPose instead of individually getx gety etc
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startPose.get(0); // we could also make it more efficient by separating startPose into startX, startY, startH instead of packaging and unpacking
        } else if (index % 2 == 0) {
            return segments.get(index/2).getx(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(0);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(0);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            double slope = (interpolateEnd-interpolateStart)/(endTime-startTime);
            return slope*(t-startTime)+interpolateStart;
        }
    }

    public double gety(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startPose.get(1);
        } else if (index % 2 == 0) {
            return segments.get(index/2).gety(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(1);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(1);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            double slope = (interpolateEnd-interpolateStart)/(endTime-startTime);
            return slope*(t-startTime)+interpolateStart;
        }
    }

    public double geth(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startPose.get(2);
        } else if (index % 2 == 0) {
            return segments.get(index/2).geth(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(2);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(2);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            double slope = (interpolateEnd-interpolateStart)/(endTime-startTime);
            return slope*(t-startTime)+interpolateStart;
        }
    }

    public VectorF getPose(double t){

        return new VectorF((float)getx(t), (float)gety(t), (float)geth(t));

    }

    public double getdx(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startDerivative.get(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdx(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(0);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(0);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            return (interpolateEnd-interpolateStart)/(endTime-startTime);
        }
    }

    public double getdy(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startDerivative.get(1);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdy(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(1);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(1);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            return (interpolateEnd-interpolateStart)/(endTime-startTime);
        }
    }

    public double getdh(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).startDerivative.get(2);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdh(t);
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(2);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(2);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            return (interpolateEnd-interpolateStart)/(endTime-startTime);
        }
    }

    public VectorF getDerivative(double t){

        return new VectorF((float)getdx(t), (float)getdy(t), (float)getdh(t));

    }

}
