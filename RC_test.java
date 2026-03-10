/*
Copyright 2026 FIRST Tech Challenge Team 293

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class Opmodetest extends LinearOpMode {

    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RB = null;

    private ElapsedTime runtime = new ElapsedTime();

    double dt = 0.02;

    //@Override
    public void runOpMode() {

        LF = hardwareMap.get(DcMotorEx.class, "DriveLF");
        RF = hardwareMap.get(DcMotorEx.class, "DriveRF");
        LB = hardwareMap.get(DcMotorEx.class, "DriveLB");
        RB = hardwareMap.get(DcMotorEx.class, "DriveRB");

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        RF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.FORWARD);

        MotorController LFcontrol = new MotorController(LF, 0.01, 0, 0, 0.020, 600);
        MotorController RFcontrol = new MotorController(RF, 0.01, 0, 0, 0.020, 600);
        MotorController LBcontrol = new MotorController(LB, 0.01, 0, 0, 0.020, 600);
        MotorController RBcontrol = new MotorController(RB, 0.01, 0, 0, 0.020, 600);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("target rad/s", "%4.2f, %4.2f, %4.2f, %4.2f", LFcontrol.getAngularTarget(), RFcontrol.getAngularTarget(), LBcontrol.getAngularTarget(), RBcontrol.getAngularTarget());
            telemetry.addData("rad/s", "%4.2f, %4.2f, %4.2f, %4.2f", LFcontrol.getAngularVel(), RFcontrol.getAngularVel(), LBcontrol.getAngularVel(), RBcontrol.getAngularVel());
            telemetry.addData("error", "%4.2f", LFcontrol.getError());
            telemetry.addData("PID", "%4.2f", LFcontrol.PID());
            telemetry.update();

            LFcontrol.setTargetRPM(180);
            RFcontrol.setTargetRPM(-180);
            LBcontrol.setTargetRPM(180);
            RBcontrol.setTargetRPM(-180);

            if (runtime.seconds() >= dt) {
                LFcontrol.update();
                RFcontrol.update();
                LBcontrol.update();
                RBcontrol.update();
                runtime.reset();
            }

        }
    }
}