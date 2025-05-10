package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.localizer.SparkFunOTOSLocalizer;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@Disabled
public abstract class AbstractAuto extends LinearOpMode {

    protected final void update() {
        robot.readSensors();
        robot.run();
        robot.printTelemetry();
        mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
        mTelemetry.update();
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        onInit();
        configure();
        Action action = onRun();

        if (isStopRequested()) return;

        waitForStart();

        resetRuntime();
        robot.drivetrain.pose = getStartPose();

        Actions.runBlocking(
                new ParallelAction(
                        action,
                        new org.firstinspires.ftc.teamcode.auto.Actions.RunnableAction(() -> {
                            update();
                            return opModeIsActive();
                        })
                )
        );

        Common.AUTO_END_POSE = robot.drivetrain.pose;
    }

    protected void onInit() {}
    protected void configure() {}
    protected abstract Pose2d getStartPose();
    protected abstract Action onRun();
}