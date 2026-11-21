
package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "SohamDrive TeleOp")
public class SohamDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Hardware
        DcMotor DriveLF = hardwareMap.get(
                DcMotor.class, "DriveLF");
        DriveLF.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor DriveRF = hardwareMap.get(
                DcMotor.class, "DriveRF");
        DcMotor DriveLB = hardwareMap.get(
                DcMotor.class, "DriveLB");
        DriveLB.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor DriveRB = hardwareMap.get(
                DcMotor.class, "DriveRB");

        DcMotor intakeMotor = hardwareMap.get(
                DcMotor.class, "intakeMotor");

        DcMotorEx launcherMotor = hardwareMap.get(
                DcMotorEx.class, "launch");
        double P = 2.5;
        double I = 0;
        double D = 0;
        double F = 0.05*(6000.0/12.0); //load factor * (max rpm / max voltage) * 1/2
        PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

        CRServo winchServo = hardwareMap.get(
                CRServo.class, "intakeWinch");

        Servo pollenGate = hardwareMap.get(
                Servo.class, "pollenGate");

        Servo nectarGate = hardwareMap.get(
                Servo.class, "nectarGate");
        //nectarGate.setDirection(DcMotorSimple.Direction.REVERSE);

        // Launcher toggle state
        boolean launcherOn = false;
        boolean previousX = false;

        // Gate positions (adjust to match your mechanism)
        double GATE_OPEN = 0.25;
        double GATE_CLOSED = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double launchRPM = 1500;

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();

        double lastTime = 0;
        double lastPos = 0;
        double rpms = 0;

        while (opModeIsActive()) {

            // -------------------------
            // Drive
            // -------------------------

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Replace this with your existing mecanum drive method
            // if you want to use MecanumRobotController.
            double lf = forward + strafe + turn;
            double rf = forward - strafe - turn;
            double lb = forward - strafe + turn;
            double rb = forward + strafe - turn;

            double max = Math.max(1.0,
                    Math.max(
                            Math.max(Math.abs(lf), Math.abs(rf)),
                            Math.max(Math.abs(lb), Math.abs(rb))
                    ));

            // Normalize wheel powers
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;

            DriveLF.setPower(lf);
            DriveRF.setPower(rf);
            DriveLB.setPower(lb);
            DriveRB.setPower(rb);

            // -------------------------
            // Intake
            // -------------------------

            if (gamepad1.dpad_up) {
                intakeMotor.setPower(1.0);
            }
            else if (gamepad1.dpad_down) {
                intakeMotor.setPower(-1.0);
            }
            else if (gamepad1.y) {
                intakeMotor.setPower(0);
            }

            // -------------------------
            // Winch
            // -------------------------

            if (gamepad1.a) {
                winchServo.setPower(1.0);
            }
            else if (gamepad1.b) {
                winchServo.setPower(-1.0);
            }
            else {
                winchServo.setPower(0);
            }

            // -------------------------
            // Launcher toggle
            // -------------------------

            if(gamepad1.dpad_left){launchRPM-=1;}
            if(gamepad1.dpad_right){launchRPM+=1;}

            if (gamepad1.x && !previousX) {
                launcherOn = !launcherOn;
            }

            previousX = gamepad1.x;

            if(launcherOn){
                launcherMotor.setVelocity(launchRPM*30/Math.PI, AngleUnit.RADIANS);
            }
            else{launcherMotor.setVelocity(0);}

            // -------------------------
            // Gates
            // -------------------------

            if(gamepad1.left_trigger>0.5){
                pollenGate.setPosition(GATE_OPEN);
            }
            else{pollenGate.setPosition(GATE_CLOSED);}
            if (gamepad1.right_trigger>0.5){
                nectarGate.setPosition(1-GATE_OPEN);
            }
            else{nectarGate.setPosition(1-GATE_CLOSED);}

            telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeMotor.getPower());
            telemetry.addData("Winch", winchServo.getPower());
            telemetry.addData("Power", launchRPM);
            telemetry.addData("RPM", rpms);
            rpms = 0.95*rpms+0.05*(launcherMotor.getCurrentPosition()-lastPos)/(runtime.seconds()-lastTime);
            lastPos=launcherMotor.getCurrentPosition();
            lastTime=runtime.seconds();

            telemetry.update();
        }

        // Stop motors and servos safely
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
        winchServo.setPower(0);
    }
}
