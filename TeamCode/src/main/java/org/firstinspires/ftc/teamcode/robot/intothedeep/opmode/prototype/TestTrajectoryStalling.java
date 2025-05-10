package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.Actions;

@Disabled
public final class TestTrajectoryStalling /*extends AbstractAuto*/ {
//    @Override
//    protected Pose2d getStartPose() {
//        return new Pose2d(0, 0, 0);
//    }
//
//    @Override
//    protected Action onRun() {
//        return waitForPress();
//    }
//
//    // Example
//    private Action waitForPress() {
//        return csRobot.drivetrain.actionBuilder(getStartPose())
//                .lineToX(10)
//                .stopAndAdd(new Actions.RunnableAction(() -> {
//                    org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.gamepadEx1.readButtons();
//                    return !MainTeleOp.gamepadEx1.isDown(GamepadKeys.Button.A);
//                }))
//                .lineToX(0)
//                .build();
//    }
}
