package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Arc extends Segment {

    VectorF center = new VectorF(0, 0, 0);
    double angleChange = 0;
    double initialAngle = 0;
    VectorF startPose = new VectorF(0, 0, 0);
    VectorF endPose = null;
    double radius = 1;



    private Arc(VectorF _startPose, VectorF _endPose, VectorF _center, double _radius, double _initialAngle, double _angleChange, double _duration) {
        super(_duration);
        startPose = _startPose;
        endPose = _endPose;
        center = _center;
        radius = _radius;
        initialAngle = _initialAngle;
        angleChange = _angleChange;
    }

    private Arc(VectorF _startPose, VectorF _center, double _radius, double _initialAngle, double _angleChange, double _duration) {
        super(_duration);
        startPose = _startPose;
        center = _center;
        radius = _radius;
        initialAngle = _initialAngle;
        angleChange = _angleChange;
    }

    public static Arc fromCenterAndAngle(VectorF _startPose, VectorF _center, double _angleChange, double _duration) {
        VectorF difference = _startPose.subtracted(_center);
        difference.put(2, 0);
        double _radius = difference.magnitude();
        double _initialAngle = Math.atan2(difference.get(1), difference.get(0));
        return new Arc(_startPose, _center, _radius, _initialAngle, _angleChange, _duration);
    }

    public static Arc fromCenterAndDegrees(VectorF _startPose, VectorF _center, double _degrees, double _duration) {
        return fromCenterAndAngle(_startPose, _center,  _degrees*Math.PI/180, _duration);
    }

    public static Arc from3points(VectorF _startPose, VectorF _throughPose, VectorF _endPose, double _duration) {

        double x0 = _startPose.get(0),   y0 = _startPose.get(1);
        double x1 = _throughPose.get(0), y1 = _throughPose.get(1);
        double x2 = _endPose.get(0),     y2 = _endPose.get(1);

        //determinant to check if collinear
        double det = 2 * (x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1));
        if (Math.abs(det) < 1e-6) {
            throw new IllegalArgumentException("Arc from3points cannot use 3 collinear points. Use a Line instead.");
        }

        //circumcenter of 3 points
        double sq0 = x0*x0 + y0*y0;
        double sq1 = x1*x1 + y1*y1;
        double sq2 = x2*x2 + y2*y2;

        double cx = (sq0 * (y1 - y2) + sq1 * (y2 - y0) + sq2 * (y0 - y1)) / det;
        double cy = (sq0 * (x2 - x1) + sq1 * (x0 - x2) + sq2 * (x1 - x0)) / det;
        VectorF _center = new VectorF((float) cx, (float) cy, 0);

        double _radius = Math.sqrt((x0 - cx)*(x0 - cx) + (y0 - cy)*(y0 - cy));

        //angles from center to each point
        double a0 = Math.atan2(y0 - cy, x0 - cx); //initialAngle
        double a1 = Math.atan2(y1 - cy, x1 - cx);
        double a2 = Math.atan2(y2 - cy, x2 - cx);

        double delta01 = a1 - a0;
        double delta12 = a2 - a1;

        //normalize differences to (-pi, pi) to handle the wrap-around
        delta01 = Math.atan2(Math.sin(delta01), Math.cos(delta01));
        delta12 = Math.atan2(Math.sin(delta12), Math.cos(delta12));

        double _angleChange = delta01 + delta12;

        return new Arc(_startPose, _endPose, _center, _radius, a0, _angleChange, _duration);
    }

    private double evaluateX(double t){
        return center.get(0)+radius*Math.cos(initialAngle+(angleChange/duration)*t)+origin.get(0);
    }
    private double evaluateY(double t){
        return center.get(1)+radius*Math.sin(initialAngle+(angleChange/duration)*t)+origin.get(1);
    }
    private double evaluateH(double t){
        if(endPose == null) {
            return startPose.get(2) + (angleChange / duration) * t;
        }
        else {
            return startPose.get(2) + ((endPose.get(2)-startPose.get(2))/ duration) * t;
        }
    }

    private double evaluateDx(double t){
        return -1*(angleChange/duration)*radius*Math.sin(initialAngle+(angleChange/duration)*t);
    }
    private double evaluateDy(double t){
        return (angleChange/duration)*radius*Math.cos(initialAngle+(angleChange/duration)*t);
    }
    private double evaluateDh(double t){
        if(endPose == null){
            return (angleChange/duration);
        }
        else{
            return (endPose.get(2)-startPose.get(2))/duration;
        }
    }
    @Override
    public VectorF getPosition(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));
        return new VectorF(
                (float)evaluateX(t),
                (float)evaluateY(t),
                (float)evaluateH(t)
        );
    }


    @Override
    public VectorF getVelocity(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));
        return new VectorF(
                (float)evaluateDx(t),
                (float)evaluateDy(t),
                (float)evaluateDh(t)
        );
    }

}
