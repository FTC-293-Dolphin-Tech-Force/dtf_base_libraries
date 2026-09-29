package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Bezier extends Spline {

    public static double smoothTime = 0.2; //for autoAlignHeading
    public Bezier(VectorF startPose, VectorF endPose, VectorF startVel, VectorF endVel, double duration) {
        super(
                calculateCoefficients(startPose.get(0), endPose.get(0), startVel.get(0), endVel.get(0), duration), //x coefficients
                calculateCoefficients(startPose.get(1), endPose.get(1), startVel.get(1), endVel.get(1), duration), //y coefficients
                calculateCoefficients(startPose.get(2), unwrapHeading(startPose.get(2), endPose.get(2)), startVel.get(2), endVel.get(2), duration), //h coefficients
                duration
        );
    }

    //aligns heading in direction of motion if you give 2D velocity vectors as inputs instead of 3d
    public Bezier(VectorF startPose, VectorF endPose, VectorF startVel2D, VectorF endVel2D, double duration, boolean autoAlignHeading) {

        //dummy call to super() so H can be fixed later
        super(  calculateCoefficients(startPose.get(0), endPose.get(0), startVel2D.get(0), endVel2D.get(0), duration),
                calculateCoefficients(startPose.get(1), endPose.get(1), startVel2D.get(1), endVel2D.get(1), duration),
                new double[4], duration);

        if (autoAlignHeading) {
            double vx0 = startVel2D.get(0);
            double vy0 = startVel2D.get(1);
            double vx1 = endVel2D.get(0);
            double vy1 = endVel2D.get(1);

            //calculate heading from velocity
            double h0 = Math.atan2(vy0, vx0);
            double targetH = Math.atan2(vy1, vx1);
            double h1 = unwrapHeading(h0, targetH);

            double v0sq = (vx0 * vx0) + (vy0 * vy0);
            double v1sq = (vx1 * vx1) + (vy1 * vy1);
            double ax0 = 2.0 * X[2];
            double ay0 = 2.0 * Y[2];
            double ax1 = 2.0 * X[2] + 6.0 * X[3] * duration;
            double ay1 = 2.0 * Y[2] + 6.0 * Y[3] * duration;

            double vh0 = 0;
            double vh1 = 0;
            if(v0sq > 1e-6){ //avoid dividing by 0
                vh0 = (vx0*ay0 - vy0*ax0)/v0sq;
            }
            if(v1sq > 1e-6){
                vh1 = (vx1*ay1 - vy1*ax1)/v1sq;
            }

            // 5. Generate final angular path matrix
            H = calculateCoefficients(h0, h1, vh0, vh1, duration);
        } else {
            // Fallback: If autoAlign is disabled, fall back to linear interpolation between given angles
            H = calculateCoefficients(startPose.get(2), unwrapHeading(startPose.get(2), endPose.get(2)), 0.0, 0.0, duration);
        }
    }

    //calculates cubic Bezier components for a Spline
    private static double[] calculateCoefficients(double start, double end, double startDerivative, double endDerivative, double duration) {
        // intermediate Bezier control points scaled by duration
        double control1 = start + (duration/3) * startDerivative;
        double control2 = end - (duration/3) * endDerivative;

        // cubic polynomial coefficients A + Bt + Ctt + Dttt
        double A = start;
        double B = (3*(control1 - start))/duration;
        double C = (3*(control2 - 2*control1 + start))/Math.pow(duration, 2);
        double D = (end - 3*control2 + 3*control1 - start)/Math.pow(duration, 3);

        return new double[]{A, B, C, D};
    }

    //if heading crosses the angle wrap boundary, this function ensures the robot takes the shorter path.
    private static double unwrapHeading(double targetFrom, double targetTo) {
        double delta = targetTo - targetFrom;
        while (delta > Math.PI)  delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;
        return targetFrom + delta;
    }
}