package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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


    MecanumRobotController(HardwareMap hardwareMap, String[] motorNames, boolean[] _reversedList, PIDFCoefficients[] _PIDList, double _dt, double[] _dimensionsLWR) {
        super(4, hardwareMap, motorNames, _reversedList, _PIDList, new int[]{250, 250, 250, 250}, _dt); //dont need counts per rev

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

    public void setTargetVelocity(VectorF velocities){

        VectorF angularVels = iK(velocities);
        for(int i = 0; i < 4; i++){

            motors[i].setVelocity(angularVels.get(i), AngleUnit.RADIANS);

        }

    }

}
