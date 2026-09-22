package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;

public class SegmentedPath extends Path {

    private final ArrayList<Segment> segments;

    private final ArrayList<Double> startTimes = new ArrayList<>();
    private final ArrayList<Double> endTimes = new ArrayList<>();

    private final int N;


    public SegmentedPath(ArrayList<Segment> _segments) {

        if (_segments == null || _segments.isEmpty()) {throw new IllegalArgumentException("SegmentedPath cannot have zero Segments.");
        }

        segments = _segments;
        N = segments.size();

        double currentTime = 0;

        for (Segment segment : segments) {

            startTimes.add(currentTime);

            currentTime += segment.getDuration();

            endTimes.add(currentTime);
        }
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