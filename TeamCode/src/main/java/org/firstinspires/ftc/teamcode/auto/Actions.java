package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.concurrent.Callable;

public final class Actions {
    public static class RunnableAction implements Action {
        private final Callable<Boolean> action;

        public RunnableAction(Callable<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                return action.call();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class SingleCheckAction implements Action {
        private final Callable<Boolean> check;
        private final Action action;
        private boolean expired = false;

        public SingleCheckAction(Callable<Boolean> check, Action action) {
            this.check = check;
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                if (expired || check.call()) {
                    expired = true;
                    return action.run(packet);
                }
                return false;
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class CallbackAction implements Action {
        private final Action action;

        private boolean isCalled = false;

        public CallbackAction(Action action, PathChain pathChain, double startCondition, int index) {
            this.action = action;
            pathChain.setCallbacks(new PathCallback(startCondition, () -> isCalled = true, PathCallback.PARAMETRIC, index));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isCalled) return true;

            action.run(telemetryPacket);
            return false;
        }
    }
}
