package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
public class MotorController {

    int countsPerRev; //encoder ticks per revolution
    double kP, kI, kD; //PID gains
    double dt;
    DcMotorEx motor;
    private ElapsedTime runtime;
    double angularVel = 0; //rad/s
    double angularTarget = 0; //rad/s
    double error = 0; double derivative = 0; double integral = 0;
    double lastError = 0; double lastTime = 0;
    double u = 0;
    double encoderCount;

    public MotorController(DcMotorEx motorSetup, double kPSetup, double kISetup, double kDSetup, double dtSetup, int countsPerRevSetup){
        motor = motorSetup;
        runtime = new ElapsedTime();
        kP = kPSetup; kI = kISetup; kD = kDSetup;
        dt = dtSetup;
        countsPerRev = countsPerRevSetup;
    }
    public double getEncoderCount(){
        encoderCount = motor.getCurrentPosition();
        return encoderCount;
    }

    public double getTimeSeconds(){
        return runtime.seconds();
    }

    public void updateRate(){
        if(getTimeSeconds()>=dt){
            angularVel = (getEncoderCount()/getTimeSeconds())/countsPerRev*2*Math.PI; //rads/sec
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
        }
    }

    public double getAngularVel(){
        return angularVel;
    }

    public double getError(){
        return error;
    }

    public double getAngularTarget(){
        return angularTarget;
    }

    public void setPower(double power){
        double u = Math.min(Math.max(power, -1), 1);
        motor.setPower(u);
    }

    public double PID(){
        error = angularTarget - angularVel;
        double time = getTimeSeconds();
        double dt = time-lastTime;
        derivative = (error-lastError)/dt;
        integral += error*dt;

        double P = kP*error;
        double I = kI*integral;
        double D = kD*derivative;

        lastError = error;
        lastTime = time;

        return P+I+D;
    }

    public void setTarget(double target){ //setSetPoint
        angularTarget = target;
    }

    public void setTargetRPM(double target){ //setSetPoint
        angularTarget = target*(2*Math.PI/60);
    }

    public void update(){
        updateRate();
        u += PID();
        setPower(u);
    }

}
