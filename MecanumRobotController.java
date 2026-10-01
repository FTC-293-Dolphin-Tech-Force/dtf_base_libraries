package org.firstinspires.ftc.teamcode.dtf_base_libraries;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumRobotController extends RobotController{
    VectorF smoothTargetPose = new VectorF(0, 0, 0); //TODO: this has to do with the smoothed pid
    VectorF smoothTargetVel = new VectorF(0, 0, 0); //TODO: this has to do with the smoothed pid
    float smoothingConstant = 0.5F; //0 means no smoothing, 1 means so smooth its actually bad
    //ElapsedTime runtime;
    double time = 0, lastTime = 0;

    float L;
    float W;
    float R;
    float[] iKmatValues;
    MatrixF iKmat; //inverse kinematics matrix

    VectorF error = new VectorF(0, 0, 0), iError = new VectorF(0, 0, 0), dError = new VectorF(0, 0, 0);
    float kP = (float) 0.60, kI = (float) 0.0, kD = (float) 0.12; //P = 0.105 I = 0.0036 D = 0.037
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


    public MecanumRobotController(HardwareMap hardwareMap, ElapsedTime _runtime, String[] motorNames, boolean[] _reversedList, PIDFCoefficients[] _PIDList, double _dt, double[] _dimensionsLWR, Localizer _localizer) {
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

        targetVel = velocity;
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

        this.targetPose = targetPose;
        setTargetVelocity(rotate(PID(targetPose, currentTime), -localizer.getHeading()));

    }



    //might not need this anymore bc its part of the superclass
    public void setTargetState(VectorF targetPose, VectorF targetDerivative, double currentTime){ //need to add targetDerivative to the velocity output

        this.targetPose = targetPose;
        //setTargetVelocity(rotate(PID(targetPose, currentTime), -localizer.getHeading()));
        setTargetVelocity(rotate(PID(targetPose, targetDerivative, currentTime), -localizer.getHeading()));
        //setTargetVelocity(rotate(PID(targetPose, currentTime).added(targetDerivative), -localizer.getHeading()));
        // i think the third line should be correct but for some reason the targetDerivative term is way too strong. The first line works though

    }

    public void followPath(Segment path, double currentTime, double initialTime){ //TODO: the BehaviorManager should be handling this, not the MecanumRobotController

        double t = currentTime-initialTime;
        setTargetState(path.getPosition(t), path.getVelocity(t), t);

    }
    public void followPath(Segment path, double t){  //TODO: the BehaviorManager should be handling this, not the MecanumRobotController

        setTargetState(path.getPosition(t), path.getVelocity(t), t);

    }



    public float getPoseErrorScalar(VectorF targetPose){

        VectorF vectorError = localizer.getPoseError(targetPose);
        float xErr = vectorError.get(0);
        float yErr = vectorError.get(1);
        float hErr = vectorError.get(2);
        return (float) Math.sqrt((xErr*xErr)+(yErr*yErr)+(hErr*hErr)*(L*L+W*W));

    }

    public float scaleErrorD(float error){

        return (float) (1.0/(error*error + 1.0));

    }

    public MatrixF getScaledDMat(VectorF error){ //generates matrix based on errors to scale the D component of the PID controller and make sure D doesn't play a role until r is small.

        float X = scaleErrorD(error.get(0));
        float Y = scaleErrorD(error.get(1));
        float H = scaleErrorD(error.get(2));

        float[] scaledDValues = new float[]{
                X, 0, 0,
                0, Y, 0,
                0, 0, H
        };

        return new GeneralMatrixF(3, 3, scaledDValues); //inverse kinematics

    }

    public VectorF PID(VectorF targetPose, double currentTime){

        error = localizer.getPoseError(targetPose);
        //double dtime = currentTime - lastTime; // final minus initial
        //lastTime = currentTime;

        dError = localizer.getVel(); // technically it should be robot vel minus vel of targetPose, but we assume targetPose is const for now.
        //iError.add(error.multiplied((float) dtime));

        /*if(Math.abs(error.get(0)) < 0.01){ // integral wipe
            iError.put(0, (float)0);
        }
        if(Math.abs(error.get(1)) < 0.01){
            iError.put(1, (float)0);
        }
        if(Math.abs(error.get(2))*Math.sqrt((L*L)+(W*W)) < 0.01){
            iError.put(2, (float)0);
        }*/

        VectorF P = kPmat.multiplied(error);
        //VectorF I = kImat.multiplied(iError);
        VectorF D = kDmat.multiplied(getScaledDMat(error).multiplied(dError)); //might need to swap error and dError

        return P.added(D).multiplied(-1);
        //return P.added(I).added(D).multiplied(-1); //idk if this is the right spot

    }

    public VectorF PID(VectorF targetPose, VectorF targetVel, double currentTime){

        float dtime = (float)(currentTime - lastTime); // final minus initial
        lastTime = currentTime;

        //exponential smoothing
        //TODO: make sure this is actually good vv
        smoothTargetPose = smoothTargetPose.multiplied(smoothingConstant*dtime).added(targetPose.multiplied(1-smoothingConstant*dtime));
        smoothTargetVel = smoothTargetVel.multiplied(smoothingConstant*dtime).added(targetVel.multiplied(1-smoothingConstant*dtime));

        error = localizer.getPoseError(smoothTargetPose);

        dError = localizer.getVel().subtracted(smoothTargetVel); // technically it should be robot vel minus vel of targetPose, but we assume targetPose is const for now.
        //iError.add(error.multiplied((float) dtime));

        /*if(Math.abs(error.get(0)) < 0.01){ // integral wipe
            iError.put(0, (float)0);
        }
        if(Math.abs(error.get(1)) < 0.01){
            iError.put(1, (float)0);
        }
        if(Math.abs(error.get(2))*Math.sqrt((L*L)+(W*W)) < 0.01){
            iError.put(2, (float)0);
        }*/

        VectorF P = kPmat.multiplied(error);
        //VectorF I = kImat.multiplied(iError);
        VectorF D = kDmat.multiplied(getScaledDMat(error).multiplied(dError)); //might need to swap error and dError

        return P.added(D).multiplied(-1);
        //return P.added(I).added(D).multiplied(-1); //idk if this is the right spot for the minus sign

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
