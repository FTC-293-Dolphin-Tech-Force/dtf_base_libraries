/*package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import java.util.ArrayList;

//@Deprecated
public class SegmentedPathOld extends PathOld {

    ArrayList<Segment> segments;
    int N;
    ArrayList<Double> startTimes = new ArrayList<Double>();
    ArrayList<Double> endTimes = new ArrayList<Double>();
    int index = 0; //if index = 2k, we are on segment k. If index = 2k+1, we are between segments k and k+1. if index = -1, we are waiting to start segment 0.
    public SegmentedPathOld(ArrayList<Segment> _segments){

        segments = _segments;
        N = segments.size();
        double currentSimulatedTime = 0;
        for(int i = 0; i < N; i++){
            startTimes.add(currentSimulatedTime);
            currentSimulatedTime += _segments.get(i).getDuration();
            endTimes.add(currentSimulatedTime);
        }

    }

    public void updateIndex(double currentTime){ //state machine (tells us which segment to be following given a time)

        for(int i = 0; i < N; i++){

            if(currentTime > startTimes.get(i)){
                if(currentTime < endTimes.get(i)){
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
            return segments.get(0).getx(0); // we could also make it more efficient by separating startPose into startX, startY, startH instead of packaging and unpacking
        } else if (index % 2 == 0) {
            return segments.get(index/2).getx(t-startTimes.get(index/2));
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
            return segments.get(0).gety(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).gety(t-startTimes.get(index/2));
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
            return segments.get(0).geth(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).geth(t-startTimes.get(index/2));
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

    public double getdx(double t){
        updateIndex(t);
        if(index == -1){
            return segments.get(0).getdx(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdx(t-startTimes.get(index/2));
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
            return segments.get(0).getdy(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdy(t-startTimes.get(index/2));
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
            return segments.get(0).getdh(0);
        } else if (index % 2 == 0) {
            return segments.get(index/2).getdh(t-startTimes.get(index/2));
        }
        else{
            double interpolateStart = segments.get((index-1)/2).endPose.get(2);
            double interpolateEnd = segments.get((index+1)/2).startPose.get(2);
            double startTime = segments.get((index-1)/2).endTime;
            double endTime = segments.get((index-1)/2).startTime;
            return (interpolateEnd-interpolateStart)/(endTime-startTime);
        }
    }

}
*/