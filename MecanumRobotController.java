package org.firstinspires.ftc.teamcode.dtf_base_libraries;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumRobotController extends RobotController{

    float L;
    float W;
    float R;
    float[] iKmatValues;
    MatrixF iKmat; //inverse kinematics matrix

    VectorF error = new VectorF(0, 0, 0), iError = new VectorF(0, 0, 0), dError = new VectorF(0, 0, 0);
    ElapsedTime runtime;
    double time = 0, lastTime = 0;
    float kP = (float) 0.25, kI = (float) 0, kD = (float) 0;


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



    public VectorF PID(VectorF targetPose, double currentTime){

        error = localizer.getPoseError(targetPose);
        double dtime = currentTime - lastTime; // final minus initial
        lastTime = currentTime;

        dError = localizer.getVel().multiplied(-1); // technically it should be vel of targetPose minus robot vel, but we assume targetPose is const for now.
        iError.add(error.multiplied((float) dtime));

        VectorF P = error.multiplied(kP);
        VectorF I = iError.multiplied(kI);
        VectorF D = dError.multiplied(kD);

        return P.added(I).added(D);

    }

    public void setkP(float value){kP = value;}
    public void setkI(float value){kI = value;}
    public void setkD(float value){kD = value;}


}
