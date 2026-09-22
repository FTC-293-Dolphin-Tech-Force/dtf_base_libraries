package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotController {

    int n;
    public DcMotorEx[] motors;
    boolean[] reversedList;
    PIDFCoefficients[] PIDList;
    int[] countsPerRevList;
    double dt;

    ElapsedTime runtime;

    // Define the orientation of the Control Hub on the robot
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    );
    IMU imu;

    YawPitchRollAngles angles;
    double heading;

    Localizer localizer;


    RobotController(int _n, HardwareMap hardwareMap, ElapsedTime _runtime, String[] hardwareNames, boolean[] _reversedList, PIDFCoefficients[] _PIDList, int[] _countsPerRevList, double _dt, Localizer _localizer){

        n = _n;
        dt = _dt;

        motors = new DcMotorEx[n];
        reversedList = new boolean[n];
        PIDList = new PIDFCoefficients[n];
        countsPerRevList = new int[n];

        runtime = _runtime;

        for(int i = 0; i < n; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, hardwareNames[i]);
            PIDList[i] = _PIDList[i];
            countsPerRevList[i] = _countsPerRevList[i];
        }

        reversedList = _reversedList;
        for(int i = 0; i < n; i++){
            setUpMotor(motors[i], reversedList[i], PIDList[i]);
        }

        imu = hardwareMap.get(IMU.class, "imu"); //make sure your imu is called "imu" in your hardware map
        imu.initialize(new IMU.Parameters(orientationOnRobot)); //make sure your control hub orientation matches this orientation

        angles = imu.getRobotYawPitchRollAngles();
        heading = angles.getYaw(AngleUnit.RADIANS);

        localizer = _localizer;

    }

    public void setUpMotor(DcMotorEx motor, boolean isReversed, PIDFCoefficients _PIDList){

        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if(isReversed){motor.setDirection(DcMotorEx.Direction.REVERSE);}
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, _PIDList);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public double getHeading(AngleUnit angleUnit){

        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

    }

    public MatrixF getRotationMat(double radians){

        float sin = (float)Math.sin(radians);
        float cos = (float)Math.cos(radians);
        float[] matValues = new float[]{
                cos, -sin, 0,
                sin,  cos, 0,
                0,    0,   1
        };
        return new GeneralMatrixF(3, 3, matValues);

    }

    public VectorF rotate(VectorF velocities, double radians){ //inverse kinematics; vx, vy, ωz -> ω0, ω1, ω2, ω3
        return getRotationMat(radians).multiplied(velocities);
    }

    public DcMotorEx[] getMotorList(){

        return motors;

    }

    public DcMotorEx getMotor(int index){

        return motors[index];

    }

    public Localizer getLocalizer(){

        return localizer;

    }

    public void setTargetVelocity(){

    }

    public void setTargetPosition(){

    }

}