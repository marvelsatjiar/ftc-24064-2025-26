package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.auto.Actions;

import java.util.LinkedList;
import java.util.Queue;

// Inspired by QC 21229
// An alternative to Actions.runBlocking() that doesn't block. Main use-case is for tele-op.
// Call run() in every loop and add actions as needed.
public final class ActionScheduler {
    final Queue<Action> actions = new LinkedList<>();
    final FtcDashboard dash = FtcDashboard.getInstance();
    final Canvas canvas = new Canvas();

    private Runnable update = null;

    public void setUpdate(Runnable update) {
        this.update = update;
    }

    public void addAction(Action action) {
        actions.add(lockSubsytems());
        actions.add(action);
        actions.add(unlockSubsytems());
    }

    // Won't generate previews
    public void run() {
        if (actions.peek() != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = actions.peek().run(packet);
            dash.sendTelemetryPacket(packet);

            if (!running) {
                actions.remove();
            }
        }
    }

    public void runBlocking() {
        while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            Action currentAction = actions.peek();

            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = currentAction.run(packet);
            dash.sendTelemetryPacket(packet);

            update.run();

            if (!running) {
                actions.remove();
            }
        }
    }

    private Action lockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.isV4BLocked = true;
            robot.extendo.isLocked = true;
            robot.lift.isLocked = true;
            robot.arm.isLocked = true;
            robot.claw.isLocked = true;
            robot.intake.isRollerLocked = true;
        });
    }

    private Action unlockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.isV4BLocked = false;
            robot.extendo.isLocked = false;
            robot.lift.isLocked = false;
            robot.arm.isLocked = false;
            robot.claw.isLocked = false;
            robot.intake.isRollerLocked = false;
        });
    }
}