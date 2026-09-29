package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Spline extends Segment {

    double[] X;
    double[] Y;
    double[] H;


    public Spline(
            double[] _X,
            double[] _Y,
            double[] _H,
            double _duration
    ) {
        super(_duration);
        X = _X;
        Y = _Y;
        H = _H;
    }


    private double evaluate(double[] coefficients, double t) {

        double value = 0;

        for (int i = coefficients.length - 1; i >= 0; i--) {
            value *= t;
            value += coefficients[i];
        }

        return value;
    }


    private double evaluateDerivative(double[] coefficients, double t) {

        double value = 0;

        //Polynomial derivatives are easy
        //A + Bt + Ctt ... becomes:
        //B + 2Ct + 3Dtt ...

        for (int i = coefficients.length - 1; i >= 1; i--) {
            value *= t;
            value += i * coefficients[i];
        }

        return value;
    }


    @Override
    public VectorF getPosition(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));
        return new VectorF(
                (float)evaluate(X, t),
                (float)evaluate(Y, t),
                (float)evaluate(H, t)
        ).added(origin);
    }


    @Override
    public VectorF getVelocity(double t) {

        //minmax to make sure t is within the bounds of the Segment
        t = Math.min(duration, Math.max(t, 0));
        return new VectorF(
                (float)evaluateDerivative(X, t),
                (float)evaluateDerivative(Y, t),
                (float)evaluateDerivative(H, t)
        );
    }

}