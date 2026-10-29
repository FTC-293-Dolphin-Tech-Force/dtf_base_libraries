package org.firstinspires.ftc.teamcode.dtf_base_libraries;


import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Test.dtf_base_librariesmain.Spline;

public class ThreePointParabola extends Spline {


    VectorF startPos;
    VectorF midPos;
    VectorF endPos;

    public ThreePointParabola(VectorF _startPos, VectorF _midPos, VectorF _endPos, double _duration) {
        super(
                calculateXYH(_startPos, _midPos, _endPos, _duration),
                calculateXYH(_startPos, _midPos, _endPos, _duration),
                calculateXYH(_startPos, _midPos, _endPos, _duration),
                _duration
        );
    }

    public static double[] calculateXYH(VectorF startPos, VectorF midPos, VectorF endPos,double duration){
            double[] results = new double[3];

                results[0] = startPos.get(0);
                results[1]= (4*midPos.get(0)- 3*startPos.get(0) -endPos.get(0))/duration;
                results[2] = 2*(startPos.get(0) + endPos.get(0) - 2*midPos.get(0))/(duration*duration);
            return results;
    }



}
