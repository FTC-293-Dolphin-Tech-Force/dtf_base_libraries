package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@Deprecated
public class PathOld {

    PathOld(){

    }

    public double getx(double t){
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
