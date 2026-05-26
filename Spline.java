package org.firstinspires.ftc.teamcode.dtf_base_libraries;

public class Spline extends Path{

    int[] degree;
    double[] X;
    double[] Y;
    double[] H;

    public Spline(double[] _X, double[] _Y, double[] _H){

        X = _X;
        Y = _Y;
        H = _H;
        degree = new int[]{X.length-1, Y.length-1, H.length-1};

    }

    public double getx(double t){
        double value = 0;
        for(int i = 0; i < degree[0]+1; i++){
            value *= t;
            value += X[degree[0]-i];
        }
        return value;
    }

    public double gety(double t){
        double value = 0;
        for(int i = 0; i < degree[1]+1; i++){
            value *= t;
            value += Y[degree[1]-i];
        }
        return value;
    }

    public double geth(double t){
        double value = 0;
        for(int i = 0; i < degree[2]+1; i++){
            value *= t;
            value += H[degree[2]-i];
        }
        return value;
    }

    public double getdx(double t){
        double value = 0;
        for(int i = 0; i < degree[0]; i++){
            value *= t;
            value += ((degree[0]-i)*(X[degree[0]-i]));
        }
        return value;
    }

    public double getdy(double t){
        double value = 0;
        for(int i = 0; i < degree[1]; i++){
            value *= t;
            value += ((degree[1]-i)*(Y[degree[1]-i]));
        }
        return value;
    }

    public double getdh(double t){
        double value = 0;
        for(int i = 0; i < degree[2]; i++){
            value *= t;
            value += ((degree[2]-i)*(H[degree[2]-i]));
        }
        return value;
    }

}
