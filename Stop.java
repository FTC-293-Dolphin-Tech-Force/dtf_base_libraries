package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Stop extends Spline{
    public Stop(VectorF pose, double duration){
        super(  new double[] {pose.get(0)},
                new double[] {pose.get(1)},
                new double[] {pose.get(2)},
                duration);
    }

}
