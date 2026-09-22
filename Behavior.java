package org.firstinspires.ftc.teamcode.dtf_base_libraries;

public abstract class Behavior {

    public enum State {
        READY,
        RUNNING,
        FINISHED,
        CANCELLED
    }

    State state = State.READY;

    public final void start(RobotController robot, double currentTime) {
        if (state == State.READY) {
            state = State.RUNNING;
            onStart(robot, currentTime);
        }
    }

    public final void update(RobotController robot, double currentTime) {
        if (state == State.RUNNING) {onUpdate(robot, currentTime);}
    }

    public final void finish(RobotController robot, double currentTime) {
        if (state != State.RUNNING) {
            return;
        }

        onFinish(robot, currentTime);
        state = State.FINISHED;
    }

    public final void cancel(RobotController robot, double currentTime) {
        if (state != State.RUNNING) {
            return;
        }

        onCancel(robot, currentTime);
        state = State.CANCELLED;
    }

    abstract void onStart(
            RobotController robot,
            double currentTime
    );

    abstract void onUpdate(
            RobotController robot,
            double currentTime
    );

    void onFinish(
            RobotController robot,
            double currentTime
    ) {}

    void onCancel(
            RobotController robot,
            double currentTime
    ) {}

    public abstract boolean isFinished();

    public State getState() {
        return state;
    }
}