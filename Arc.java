package org.firstinspires.ftc.teamcode.dtf_base_libraries;

public class Arc extends Path{

    int[] degree;
    double[] X;
    double[] Y;
    double[] H;

    public Arc(){

        //X = _X;
        //Y = _Y;
        //H = _H;
        //degree = new int[]{X.length-1, Y.length-1, H.length-1};

    }

    public double getx(double t){

        return Math.cos(t);
    }

    public double gety(double t){

        return Math.sin(t);
    }

    public double geth(double t){

        return 6.2857*Math.sin(t*0.1);
    }

    public double getdx(double t){

        return -Math.sin(t);
    }

    public double getdy(double t){

        return Math.cos(t);
    }

    public double getdh(double t){

        return 0.62857*Math.cos(t*0.1);
    }

}
