package org.firstinspires.ftc.teamcode.dtf_base_libraries;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumRobotController2 extends RobotController{

    float L;
    float W;
    float R;
    float[] iKmatValues;
    MatrixF iKmat; //inverse kinematics matrix

    VectorF error = new VectorF(0, 0, 0), iError = new VectorF(0, 0, 0), dError = new VectorF(0, 0, 0);
    ElapsedTime runtime;
    double time = 0, lastTime = 0;
    float kP = (float) 0.23, kI = (float) 0.0, kD = (float) 0.0; //P = 0.105 I = 0.0036 D = 0.037
    float[] kPmatValues = new float[]{
        kP, 0, 0,
        0, kP, 0,
        0, 0, kP
    };

    float[] kImatValues = new float[]{
            kI, 0, 0,
            0, kI, 0,
            0, 0, kI
    };

    float[] kDmatValues = new float[]{
            kD, 0, 0,
            0, kD, 0,
            0, 0, kD
    };
    MatrixF kPmat = new GeneralMatrixF(3, 3, kPmatValues);
    MatrixF kImat = new GeneralMatrixF(3, 3, kImatValues);
    MatrixF kDmat = new GeneralMatrixF(3, 3, kDmatValues);


    public MecanumRobotController2(HardwareMap hardwareMap, ElapsedTime _runtime, String[] motorNames, boolean[] _reversedList, PIDFCoefficients[] _PIDList, double _dt, double[] _dimensionsLWR, Localizer _localizer) {
        super(4, hardwareMap, _runtime, motorNames, _reversedList, _PIDList, new int[]{250, 250, 250, 250}, _dt, _localizer); //dont need counts per rev

        L = (float)_dimensionsLWR[0];
        W = (float)_dimensionsLWR[1];
        R = (float)_dimensionsLWR[2];

        iKmatValues = new float[]{
            1/R, -1/R, -(L+W)/R,
            1/R,  1/R,  (L+W)/R,
            1/R,  1/R, -(L+W)/R,
            1/R, -1/R,  (L+W)/R,
        };

        iKmat = new GeneralMatrixF(4, 3, iKmatValues); //inverse kinematics
    }

    public VectorF iK(VectorF velocities){ //inverse kinematics; vx, vy, ωz -> ω0, ω1, ω2, ω3
        return iKmat.multiplied(velocities);
    }

    public void setTargetVelocity(VectorF velocity){

        VectorF angularVels = iK(velocity);
        for(int i = 0; i < 4; i++){

            motors[i].setVelocity(angularVels.get(i), AngleUnit.RADIANS);

        }

    }

    /*
    public double setTargetPosition(VectorF targetPose){ //this is super messed up; fix it later

        localizer.updatePos();
        VectorF displacement = targetPose.subtracted(localizer.getPose());
        VectorF targetVel = displacement.multiplied((float) 0.1);
        setTargetVelocity(targetVel);
        return displacement.magnitude();

    }*/

    public void setTargetPosition(VectorF targetPose, double currentTime){ //this is super messed up; fix it later

        setTargetVelocity(rotate(PID(targetPose, currentTime), -localizer.getHeading()));

    }



    public float getPoseErrorScalar(VectorF targetPose){

        VectorF vectorError = localizer.getPoseError(targetPose);
        float xErr = vectorError.get(0);
        float yErr = vectorError.get(1);
        float hErr = vectorError.get(2);
        return (float) Math.sqrt((xErr*xErr)+(yErr*yErr)+(hErr*hErr)*(L*L+W*W));

    }
    public VectorF PID(VectorF targetPose, double currentTime){

        error = localizer.getPoseError(targetPose);
        double dtime = currentTime - lastTime; // final minus initial
        lastTime = currentTime;

        dError = localizer.getVel().multiplied(-1); // technically it should be vel of targetPose minus robot vel, but we assume targetPose is const for now.
        iError.add(error.multiplied((float) dtime));

        if(Math.abs(error.get(0)) < 0.01){ // integral wipe
            iError.put(0, 0);
        }
        if(Math.abs(error.get(1)) < 0.01){
            iError.put(1, 0);
        }
        if(Math.abs(error.get(2))*Math.sqrt((L*L)+(W*W)) < 0.01){
            iError.put(2, 0);
        }

        VectorF P = kPmat.multiplied(error);
        VectorF I = kImat.multiplied(iError);
        VectorF D = kDmat.multiplied(dError);

        return P.added(I).added(D).multiplied(-1); //idk if this is the right spot

    }

    public void setPIDconstant(int i, int j, float value){ //i selects kP, kI, and kD (0-2) and j selects vx, vy, and vh (0-2)
        if(i == 0){
            kPmatValues[4*j] = value;
            kPmat = new GeneralMatrixF(3, 3, kPmatValues);
        }
        else if(i == 1){
            kImatValues[4*j] = value;
            kImat = new GeneralMatrixF(3, 3, kImatValues);
        }
        else{
            kDmatValues[4*j] = value;
            kDmat = new GeneralMatrixF(3, 3, kDmatValues);
        }
    }

    public float getPIDconstant(int i, int j){ //i selects kP, kI, and kD (0-2) and j selects vx, vy, and vh (0-2)
        float value;
        if(i == 0){
            value = kPmatValues[4*j];
        }
        else if(i == 1){
            value = kImatValues[4*j];
        }
        else{
            value = kDmatValues[4*j];
        }
        return value;
    }


}
