package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;

@TeleOp(group = "24064 Main")
public final class MainTeleOp extends LinearOpMode {
    // Gamepads and the 'robot' class is imported to save lines and to import controls
    public static GamepadEx gamepadEx1, gamepadEx2;

    // Quick method that is used for better handling the controller
    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() {
        boolean isSpecimenMode = false;
        boolean isHangControlInversed = false;

        boolean isNeutralStartPosition = true;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        Pose2d endPose = Common.AUTO_END_POSE;
        if (endPose != null) {
            robot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - Common.FORWARD);
        }


        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(A)) isNeutralStartPosition = !isNeutralStartPosition;

            mTelemetry.addLine("| A | Toggle start position");

            mTelemetry.addLine("Selected start position : " + (isNeutralStartPosition ? "Neutral" : "Wall Pickup"));
        }

        if (!isNeutralStartPosition) {

            robot.arm.setArmAngle(Arm.ArmAngle.WALL_PICKUP);
            robot.arm.setWristAngle(Arm.WristAngle.WALL_PICKUP);
            robot.arm.setArmstendoAngle(Arm.Extension.WALL_PICKUP);
            robot.claw.setAngle(Claw.ClawAngles.WALL_PICKUP);
            robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);

            robot.setCurrentState(Robot.State.WALL_PICKUP);
        } else {

            robot.actionScheduler.addAction(new SequentialAction(
                    RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                    RobotActions.setArm(Arm.ArmAngle.BASKET, 1),
                    RobotActions.setArm(Arm.ArmAngle.NEUTRAL, 0)
            ));
        }

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();


            // Gamepad 1
            double slowMult = gamepadEx1.isDown(LEFT_BUMPER) ? 0.5 : 1;
            double slowTurningMult = gamepadEx1.isDown(LEFT_BUMPER) ? 0.3 : 1;

            robot.drivetrain.setTeleOpMovementVectors(
                    gamepadEx1.getLeftY() * slowMult,
                    -gamepadEx1.getLeftX() * slowMult,
                    -gamepadEx1.getRightX() * slowTurningMult
            );

            if (keyPressed(1, B)) robot.drivetrain.setCurrentHeading(0);

            if (keyPressed(1, X)) isSpecimenMode = !isSpecimenMode;

            if (keyPressed(2, RIGHT_BUMPER)) {
                robot.lift.runManual(gamepadEx2.getLeftY() * 0.2);
                robot.lift.reset();
            } else robot.lift.runManual((0));

            switch (robot.getCurrentState()) {
                // MISC ============================================================================
                case NEUTRAL:
                    doExtendoControls();
                    doIntakeControls();

                    if ((keyPressed(1, RIGHT_BUMPER) && !isSpecimenMode) || (keyPressed(1, LEFT_STICK_BUTTON) && isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.EXTENDED));
                    if (keyPressed(2, Y) || (keyPressed(1, RIGHT_BUMPER) && isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.setupWallPickup());
                    if (keyPressed(2, X))
                        robot.actionScheduler.addAction(RobotActions.transfer());
                    if (keyPressed(2, A))
                        robot.actionScheduler.addAction(RobotActions.retractTransferAndSetupBasket());
                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.setupHang());
                    if (keyPressed(1, LEFT_STICK_BUTTON) && !isSpecimenMode)
                        robot.actionScheduler.addAction(RobotActions.retractTransferAndSetupBasket());
                    break;
                case EXTENDO_OUT:
                    doExtendoControls();
                    doIntakeControls();

                    if (keyPressed(1, RIGHT_BUMPER) && isSpecimenMode)
                        robot.actionScheduler.addAction(RobotActions.scoreSpecimen());

                    if (keyPressed(1, LEFT_STICK_BUTTON) && isSpecimenMode)
                        robot.actionScheduler.addAction(RobotActions.retractExtendo());

                    if (keyPressed(2, X))
                        robot.actionScheduler.addAction(RobotActions.transfer());
                    if (keyPressed(2, Y))
                        robot.actionScheduler.addAction(RobotActions.retractExtendo());
                    if (keyPressed(2, A) || (keyPressed(1, RIGHT_BUMPER) && !isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.retractTransferAndSetupBasket());
                    break;
                case TRANSFERRED:
                    if (keyPressed(2, A))
                        robot.actionScheduler.addAction(RobotActions.setupBasket(true));
                    if (keyPressed(2, Y))
                        robot.actionScheduler.addAction(RobotActions.setupBasket(false));
                    if (keyPressed(2, X))
                        robot.actionScheduler.addAction(RobotActions.setupDropSample());
                    if (keyPressed(2, B))
                        robot.actionScheduler.addAction(RobotActions.interleaveDropSample(RobotActions.INTERLEAVE_DROP.unclampClawForDropOffWait));
                    break;
                // BASKET ==========================================================================
                case SETUP_SCORE_BASKET:
                    robot.sweeper.setAngle(Sweeper.SweeperAngles.RETRACTED);

                    if (keyPressed(2, X) || (keyPressed(1, RIGHT_BUMPER) && !isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.scoreBasket());

                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.setupHang());
                    doIntakeControls();

                    break;
                case SCORED_BASKET:
                    if (keyPressed(2, X) || (keyPressed(1, RIGHT_BUMPER) && !isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.retractAfterScoreBasket());

                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.setupHang());

                    doExtendoControls();
                    doIntakeControls();

                    break;
                // CHAMBER =========================================================================
                case SETUP_SPECIMEN:
                    if (keyPressed(2, X) || (keyPressed(1, RIGHT_BUMPER) && isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.scoreSpecimen());

                    doExtendoControls();
                    doIntakeControls();

                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.setupHang());
                    break;
                // WALL PICKUP =====================================================================
                case WALL_PICKUP:
                    robot.sweeper.setAngle(Sweeper.SweeperAngles.RETRACTED);

                    doIntakeControls();
                    doExtendoControls();

                    if (keyPressed(2, X) || (keyPressed(1, RIGHT_BUMPER) && isSpecimenMode))
                        robot.actionScheduler.addAction(RobotActions.setupSpecimen());
                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.setupHang());
                    break;

                // HANG ============================================================================
                case SETUP_HANG:

                    robot.sweeper.setAngle(Sweeper.SweeperAngles.RETRACTED);

                    if (keyPressed(2, A)) isHangControlInversed = !isHangControlInversed;

                    double trigger1 = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                    double trigger2 = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

                    if (trigger1 >= 0.1 || trigger2 >= 0.1)
                        robot.hang.setPower(isHangControlInversed ? -trigger2 : trigger2, isHangControlInversed ? -trigger1 : trigger1);
                    else
                        robot.hang.setPower(0, 0);

                    if (keyPressed(2, LEFT_BUMPER))
                        robot.actionScheduler.addAction(RobotActions.doHang());
                    if (keyPressed(2, X))
                        robot.actionScheduler.addAction(RobotActions.retractToNeutral(0.2));
                    break;
            }

            robot.drivetrain.update();
            robot.run();
            robot.printTelemetry();
        }
    }

    public void doExtendoControls() {
//        boolean isExtendoRetracted = robot.extendo.getTargetAngle() != Extendo.LINKAGE_MIN_ANGLE;
//        boolean isV4BDown = robot.intake.getTargetV4BAngle() == Intake.V4BAngle.DOWN;
//        boolean isAllianceSpecificSample = robot.intake.getCurrentSample() == RED && IS_RED || robot.intake.getCurrentSample() == BLUE && !IS_RED;
//        boolean isSampleYellow = robot.intake.getCurrentSample() == YELLOW;
//
//        if (isExtendoRetracted && isV4BDown) {
//            if (isSampleYellow) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.transferToClaw());
//            }
//        }
//
//        if (isExtendoRetracted && isV4BDown) {
//            if (isAllianceSpecificSample) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.retractForTransfer());
//            }
//        }

        if (keyPressed(2, DPAD_UP))
            robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.EXTENDED));
        if (keyPressed(2, DPAD_LEFT))
            robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH));
        if (keyPressed(2, DPAD_DOWN))
            robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.ONE_HALF));
        if (keyPressed(2, DPAD_RIGHT))
            robot.actionScheduler.addAction(RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS));
        if (gamepadEx2.getRightY() != 0)
            robot.extendo.setTargetAngleWithStick(-gamepadEx2.getRightY());
    }

    public void doIntakeControls() {
//        boolean isIntakeAlreadyPowered = robot.intake.getRollerPower() != -1;
//        boolean isV4BDown = robot.intake.getTargetV4BAngle() == Intake.V4BAngle.DOWN;
//        boolean isOppositeAllianceSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.BLUE && IS_RED || robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.RED && !IS_RED;
//
//        if (isIntakeAlreadyPowered && isV4BDown) {
//            if (isOppositeAllianceSample) {
//                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP, true);
//                robot.actionScheduler.addAction(RobotActions.setRollers(-1, 0.4));
//            }
//        }

        if (keyPressed(1, A)) robot.sweeper.toggleSweeper();

        if (!gamepadEx1.isDown(RIGHT_BUMPER) && !gamepadEx2.isDown(RIGHT_BUMPER)) {
            double trigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (trigger != 0) {
                robot.intake.setTargetV4BAngle(Intake.V4BAngle.DOWN);
                robot.intake.setRollerPower(trigger * 0.8);
            } else {
                robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);
                robot.intake.setRollerPower(0);
            }
        }
    }
}