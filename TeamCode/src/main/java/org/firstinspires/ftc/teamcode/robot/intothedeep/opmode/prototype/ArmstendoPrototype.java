package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@TeleOp(name = "Armstendo Prototype", group = "Prototype")
public class ArmstendoPrototype extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm.Extension targetPosition = Arm.Extension.RETRACTED;
        Arm.ArmAngle targetArmAngle = Arm.ArmAngle.COLLECTING;
        Arm.WristAngle targetWristAngle = Arm.WristAngle.COLLECTING;

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx.readButtons();

            boolean isAPressed = gamepadEx.wasJustPressed(A);
            boolean isBPressed = gamepadEx.wasJustPressed(B);

            if (gamepadEx.wasJustPressed(DPAD_LEFT)) targetPosition = Arm.Extension.TRANSFER;
            if (gamepadEx.wasJustPressed(DPAD_UP)) targetPosition = Arm.Extension.EXTENDED;
            if (gamepadEx.wasJustPressed(DPAD_RIGHT)) targetPosition = Arm.Extension.WALL_PICKUP;
            if (gamepadEx.wasJustPressed(DPAD_DOWN)) targetPosition = Arm.Extension.RETRACTED;


            switch (targetArmAngle) {
                case COLLECTING:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.BASKET;
                    break;
                case BASKET:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.WALL_PICKUP;
                    break;
                case WALL_PICKUP:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.SCORE_SPECIMEN;
                    break;
                case SCORE_SPECIMEN:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.NEUTRAL;
                    break;
                case NEUTRAL:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.COLLECTING;
                    break;
            }

            switch (targetWristAngle) {
                case COLLECTING:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.BASKET;
                    break;
                case BASKET:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.WALL_PICKUP;
                    break;
                case WALL_PICKUP:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.SCORE_SPECIMEN;
                    break;
                case SCORE_SPECIMEN:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.GRAB_OFF_WALL;
                    break;
                case GRAB_OFF_WALL:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.COLLECTING;
                    break;
            }

            arm.setWristAngle(targetWristAngle);
            arm.setArmAngle(targetArmAngle);
            arm.setArmstendoAngle(targetPosition);

            arm.printTelemetry();

            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());

            mTelemetry.update();

            lift.run();
            arm.run();
        }
    }
}
