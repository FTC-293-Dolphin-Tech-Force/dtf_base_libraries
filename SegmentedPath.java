package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

public class SegmentedPath extends Segment {

    private ArrayList<Segment> segments;

    private ArrayList<Double> startTimes = new ArrayList<>();
    private ArrayList<Double> endTimes = new ArrayList<>();

    private int N;


    public SegmentedPath(ArrayList<Segment> _segments) {

        super(0);
        if (_segments == null || _segments.isEmpty()) {throw new IllegalArgumentException("SegmentedPath cannot have zero Segments.");
        }

        segments = _segments;
        N = segments.size();

        for (Segment segment : segments) {
            startTimes.add(duration); //TODO: Combine startTimes with endTimes since it's redundant
            duration += segment.getDuration();
            endTimes.add(duration);
        }
    }

    public SegmentedPath(Segment segment) { //literally just a wrapper for the Segment class,

        super(segment.getDuration());
        segments.add(segment);
        N = 1;
        startTimes.add(duration); //TODO: Combine startTimes with endTimes since it's redundant
        duration += segment.getDuration();
        endTimes.add(duration);

    }

    public void add(Segment segment){ //I can maybe overload this method to take both a segment and an index

        segments.add(segment);
        N++;
        startTimes.add(duration); //TODO: Combine startTimes with endTimes since it's redundant
        duration += segment.getDuration();
        endTimes.add(duration);

    }


    private int getSegmentIndex(double t) {

        // Before path starts
        if (t <= 0) {
            return 0;
        }

        // Find the segment containing t
        for (int i = 0; i < N; i++) {

            if (t < endTimes.get(i)) {
                return i;
            }

        }

        //in case time goes past the end
        return N - 1;
    }


    private double getLocalTime(double t, int segmentIndex) {

        double localTime = t - startTimes.get(segmentIndex);

        //clamp time
        return Math.min(segments.get(segmentIndex).getDuration(), Math.max(localTime, 0));
    }


    @Override
    public VectorF getPosition(double t) {

        int segmentIndex = getSegmentIndex(t);

        double localTime =
                getLocalTime(t, segmentIndex);

        return segments
                .get(segmentIndex)
                .getPosition(localTime);
    }


    @Override
    public VectorF getVelocity(double t) {

        int segmentIndex = getSegmentIndex(t);

        double localTime =
                getLocalTime(t, segmentIndex);

        return segments
                .get(segmentIndex)
                .getVelocity(localTime);
    }


    @Override
    public double getDuration() {

        return endTimes.get(N - 1);
    }
}