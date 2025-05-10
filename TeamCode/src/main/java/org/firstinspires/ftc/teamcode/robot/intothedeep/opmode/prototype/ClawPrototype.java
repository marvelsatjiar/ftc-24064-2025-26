package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.BulkReader;
import org.firstinspires.ftc.teamcode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

@Config
@TeleOp(name = "Claw Prototype", group = "Prototype")
public final class ClawPrototype extends LinearOpMode {
    public static double
            CLAMP_ANGLE = 7,
            DEPOSIT_ANGLE = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        SimpleServoPivot claw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "claw"));

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) claw.toggle();

            claw.updateAngles(DEPOSIT_ANGLE, CLAMP_ANGLE);

            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());

            claw.run();
        }
    }
}
