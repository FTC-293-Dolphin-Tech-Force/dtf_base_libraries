package org.firstinspires.ftc.teamcode.dtf_base_libraries;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class BehaviorManager {

    private final RobotController robot;
    private final ElapsedTime runtime;
    private final ArrayList<Behavior> activeBehaviors = new ArrayList<>();

    public BehaviorManager(RobotController _robot, ElapsedTime _runtime) {
        robot = _robot;
        runtime = _runtime;
    }


    //Add behaviors to the manager, which will start when update() is called next
    public void add(Behavior behavior) {
        if (behavior == null) {return;} //dont add null to the list of things to *do*...
        activeBehaviors.add(behavior);
    }

    //loop this in the FTC opMode
    public void update() {

        double currentTime = runtime.seconds();

        for (int i = 0; i < activeBehaviors.size(); i++) {

            Behavior behavior = activeBehaviors.get(i);

            // Start new behavior
            if (behavior.getState() == Behavior.State.READY) {
                behavior.start(robot, currentTime);
            }

            // Update running behavior
            if (behavior.getState() == Behavior.State.RUNNING) {
                behavior.update(robot, currentTime);
            }

            // Finish completed behavior
            if (behavior.getState() == Behavior.State.FINISHED || behavior.isFinished()) {

                if (behavior.getState() == Behavior.State.RUNNING) {
                    behavior.finish(robot, currentTime);
                }

                activeBehaviors.remove(i);
                i--;
            }

            // Remove cancelled behavior
            else if (behavior.getState() == Behavior.State.CANCELLED) {
                activeBehaviors.remove(i);
                i--;
            }
        }
    }

    //cancel a behavior early for whatever reason
    public void cancel(Behavior behavior) {

        if (activeBehaviors.contains(behavior)) {
            behavior.cancel(robot, runtime.seconds());
            activeBehaviors.remove(behavior);
        }
    }

    //cancel all behaviors
    public void cancelAll() {
        for (Behavior behavior : activeBehaviors) {
            if (behavior.getState() == Behavior.State.RUNNING) {
                behavior.cancel(robot, runtime.seconds());
            }
        }
        activeBehaviors.clear();
    }

    public boolean isBusy() {
        return !activeBehaviors.isEmpty();
    }

    public ArrayList<Behavior> getActiveBehaviors() {
        return new ArrayList<>(activeBehaviors);
    }
}