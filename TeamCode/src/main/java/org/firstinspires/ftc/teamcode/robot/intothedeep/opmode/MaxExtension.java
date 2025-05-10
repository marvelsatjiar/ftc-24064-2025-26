package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@TeleOp(group = "24064 Main")
public class MaxExtension extends LinearOpMode {
    public static GamepadEx gamepadEx1;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            arm.setArmAngle(Arm.ArmAngle.BASKET);
            arm.setWristAngle(Arm.WristAngle.BASKET);
            arm.setArmstendoAngle(Arm.Extension.EXTENDED);
            extendo.setTargetExtension(Extendo.Extension.EXTENDED);
            intake.setTargetV4BAngle(Intake.V4BAngle.UP);
            claw.setAngle(Claw.ClawAngles.DEPOSIT);
            lift.setTargetTicks(Lift.Ticks.HIGH_BASKET);


            double rightTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (rightTrigger != 0) {
                intake.setRollerPower(rightTrigger);
                intake.setTargetV4BAngle(Intake.V4BAngle.DOWN);
            } else intake.setRollerPower(0);

            arm.run();
            extendo.run(false);
            lift.run();
            intake.run(extendo.getTargetAngle());
            claw.run();
        }
    }
}