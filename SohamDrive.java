
package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SohamDrive TeleOp")
public class SohamDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Hardware
        DcMotor DriveLF = hardwareMap.get(
                DcMotor.class, "DriveLF");
        DcMotor DriveRF = hardwareMap.get(
                DcMotor.class, "DriveRF");
        DcMotor DriveLB = hardwareMap.get(
                DcMotor.class, "DriveLB");
        DcMotor DriveRB = hardwareMap.get(
                DcMotor.class, "DriveRB");

        DcMotor intakeMotor = hardwareMap.get(
                DcMotor.class, "intakeMotor");

        DcMotor launcherMotor = hardwareMap.get(
                DcMotor.class, "launch");

        CRServo winchServo = hardwareMap.get(
                CRServo.class, "intakeWinch");

        Servo pollenGate = hardwareMap.get(
                Servo.class, "pollenGate");

        Servo nectarGate = hardwareMap.get(
                Servo.class, "nectarGate");

        // Launcher toggle state
        boolean launcherOn = false;
        boolean previousX = false;

        // Gate positions (adjust to match your mechanism)
        final double GATE_OPEN = 1.0;
        final double GATE_CLOSED = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------
            // Drive
            // -------------------------

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

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
            else {
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

            if (gamepad1.x && !previousX) {
                launcherOn = !launcherOn;
            }

            previousX = gamepad1.x;

            launcherMotor.setPower(launcherOn ? 1.0 : 0.0);

            // -------------------------
            // Gates
            // -------------------------

            pollenGate.setPosition(
                    gamepad1.left_trigger > 0.5
                            ? GATE_OPEN
                            : GATE_CLOSED
            );

            nectarGate.setPosition(
                    gamepad1.right_trigger > 0.5
                            ? GATE_OPEN
                            : GATE_CLOSED
            );

            telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
            telemetry.addData("Intake", intakeMotor.getPower());
            telemetry.addData("Winch", winchServo.getPower());

            telemetry.update();
        }

        // Stop motors and servos safely
        intakeMotor.setPower(0);
        launcherMotor.setPower(0);
        winchServo.setPower(0);
    }
}
