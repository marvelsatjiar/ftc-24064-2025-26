package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@Disabled
public abstract class AbstractAutoPedro extends LinearOpMode {

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

        robot.actionScheduler.setUpdate(this::update);

        onInit();
        configure();

        if (isStopRequested()) return;

        waitForStart();

        resetRuntime();
        robot.drivetrain.setPose(getStartPose());

        onRun();
        Common.AUTO_END_POSE_PEDRO = robot.drivetrain.getPose();
    }

    protected void onInit() {}
    protected void configure() {}
    protected abstract Pose getStartPose();
    protected abstract void onRun();
}