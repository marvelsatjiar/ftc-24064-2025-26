package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

@TeleOp(name = "Basket Action Auto Prototype", group = "Prototype")
public class BasketActionAutoPrototype extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Common.robot = new Robot(hardwareMap);

        if (opModeInInit()) {
            robot.claw.setAngle(Claw.ClawAngles.SAMPLE_CLAMPED);
            robot.arm.setArmAngle(Arm.ArmAngle.WALL_PICKUP);
            robot.arm.setArmstendoAngle(Arm.Extension.RETRACTED);
            robot.arm.setWristAngle(Arm.WristAngle.GRAB_OFF_WALL);
            robot.extendo.setTargetExtension(Extendo.Extension.RETRACTED);
            robot.setCurrentState(Robot.State.TRANSFERRED);
            robot.intake.setTargetV4BAngle(Intake.V4BAngle.VERTICAL);

            robot.intake.run(robot.extendo.getTargetAngle());
            robot.extendo.run(false);
            robot.arm.run();
            robot.claw.run();
        }

        waitForStart();

        robot.actionScheduler.addAction(RobotActions.setupWithoutV4BBasket(true));

        while (opModeIsActive()) {
            robot.run();
            robot.readSensors();
        }
    }
}
