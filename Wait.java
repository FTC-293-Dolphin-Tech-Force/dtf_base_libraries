package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Wait extends Behavior{

    VectorF targetPose;
    @Override
    void onStart(RobotController robot, double currentTime) {
        targetPose = robot.getTargetPose();
    }

    @Override
    void onUpdate(RobotController robot, double currentTime) {
        robot.setTargetState(targetPose, new VectorF(0, 0, 0), currentTime);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
