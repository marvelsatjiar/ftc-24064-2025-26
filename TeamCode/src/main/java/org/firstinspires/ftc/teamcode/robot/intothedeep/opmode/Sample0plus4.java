package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_BLUE_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_RED_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAlignToSample;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;


@Autonomous(name ="Sample 0+4")
@Config
public class Sample0plus4 extends AbstractAuto {
    private AutoAlignToSample autoAlignToSample;

    private Action currentTraj;

    private boolean isConfirmed;

    public static class Positions {
        public double
                startingPositionX = -39,
                startingPositionY = -63.375,
                xBasket2 = -62,
                yBasket2 = -52,
                xBasket3 = -64.5,
                yBasket3 = -54.5,
                xIntakeSample3 = -48,
                yIntakeSample3 = -43,
                subX = -14,
                subY = -10,
                midSubX = -40,
                midSubY = -14,
                yBasketSub = -60.5,
                sample5thOffset = 0,
                sample6thOffset = 5,
                sample7thOffset = 10,
                sample8thOffset = 15;
    }

    public static class Headings {
        public double
                lineToSubTangent = 70,
                lineToBasketTangent = 247,
                intake1stSampleAngle = 77,
                intake2ndSampleAngle = 90,
                intake3rdSampleAngle = 145,
                dropOff4thSample = 65;
    }

    public static class Timings {
        public double
                sleepUntilLiftRetracted = 0.2,
                waitBeforeUnclampSubBasket = 0.2,
                secondsToExpire = 0.5,
                waitBefore2ndTransfer = 0.9,
                waitBefore3rdTransfer = 1,
                sleep3rdBeforeExtending = 1,
                sleep4thBeforeExtending = 2.7,
                intake4thDelayAfter3rd = 0.7,
                waitBefore3rdRetract = 0.3,
                waitBefore4thTransfer = 0.3,
                waitBefore3rdIntake = 2.2,
                extendWhile2ndTurningDelay = 1.4,
                delayBefore1stUnclamp = 1.4,
                sleepAfter1stUnclamp = 0.5,
                sleepAfter2ndUnclamp = 0.5,
                sleepBefore3rdUnclamp = 0.65,
                sleepBefore4thUnclamp = 1.3,
                timeFor2ndIntaking = 0.9;
    }

    public static class Miscellaneous {
        public double
                subVelocityConstraint = 115,
                subMinAccelConstraint = -110,
                subMaxAccelConstraint = 105,
                intakeSubVelocityConstraint = 75,
                setupIntake2ndExtendoAngle = 70,
                setupIntake3rdExtendoAngle = 80,
                setupIntake4thExtendoAngle = 90,
                intake2ndExtendoAngle = 141,
                intake3rdExtendoAngle = 127,
                intake4thExtendoAngle = 137;
    }

    public static Positions POS = new Positions();
    public static Headings HEAD = new Headings();
    public static Timings TIME = new Timings();
    public static Miscellaneous MISC = new Miscellaneous();


    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();

            if (gamepadEx1.wasJustPressed(LEFT_BUMPER) && gamepadEx1.wasJustPressed(RIGHT_BUMPER)) {
                isConfirmed = true;
                autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE);

                TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

                builder = scoreFirstFourSamples(builder);
//                builder = scoreAllSubSamples(builder);
//                builder = park(builder);

                currentTraj = builder.build();
            }
        }

        if (!isConfirmed) {
            autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE);

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

            builder = scoreFirstFourSamples(builder);
//            builder = scoreAllSubSamples(builder);
//                builder = park(builder);

            currentTraj = builder.build();
        }
    }


    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(POS.startingPositionX, POS.startingPositionY, Math.toRadians(0));
    }

    @Override
    protected void onInit() {
        autoAlignToSample = new AutoAlignToSample(robot.limelightEx);

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

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
        builder = scoreFirstFourSamples(builder);
//        builder = scoreAllSubSamples(builder);

        return builder.build();
    }


    private TrajectoryActionBuilder scoreFirstFourSamples(TrajectoryActionBuilder builder) {
        builder = builder

                // Setup Score 1st Sample Objectively + Extends for 2nd Objectively
                .afterTime(0, new ParallelAction(
                        RobotActions.setExtendo(MISC.setupIntake2ndExtendoAngle, 0),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0),
                        RobotActions.setupWithoutV4BBasket(true)
                ))
                .afterTime(TIME.extendWhile2ndTurningDelay, new ParallelAction(
                        RobotActions.setExtendo(MISC.intake2ndExtendoAngle, 0),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setRollers(0.8, 0)
                ))

                .afterTime(TIME.delayBefore1stUnclamp, new SequentialAction(
                        RobotActions.scoreBasket(),
                        new SleepAction(TIME.sleepAfter1stUnclamp),
                        RobotActions.autoRetractAfterScoreBasket()

                ))
                .strafeToLinearHeading(new Vector2d(POS.xBasket2, POS.yBasket2), Math.toRadians(HEAD.intake1stSampleAngle))

                // Score 1st Sample + Intakes 2nd Objectively TODO waitBefore2ndTransfer
                .stopAndAdd(new SequentialAction(
                        RobotActions.runRollersUntilCollected(0.8, ColorRangefinderEx.SampleColor.YELLOW, TIME.timeFor2ndIntaking, Extendo.LINKAGE_MAX_ANGLE),
                        new SleepAction(TIME.timeFor2ndIntaking),
                        new ParallelAction(
                                RobotActions.setV4B(Intake.V4BAngle.UP, 0),
                                RobotActions.setExtendo(Extendo.Extension.RETRACTED, 0)
                        )
                ))


                // Transfer & Setup 2nd Objectively + Extend for 3rd Objectively TODO sleep3rdBeforeExtending
                .afterTime(TIME.waitBefore2ndTransfer, new ParallelAction(
                        RobotActions.retractTransferAndSetupBasket(),
                        new SequentialAction(
                                new SleepAction(TIME.sleep3rdBeforeExtending),
                                RobotActions.setExtendo(MISC.setupIntake3rdExtendoAngle, 0)
                        )
                ))

                .strafeToLinearHeading(new Vector2d(POS.xBasket3, POS.yBasket3), Math.toRadians(HEAD.intake2ndSampleAngle))

                .waitSeconds(TIME.waitBefore3rdIntake)
                // Score 2nd + Intake 3rd Objectively TODO waitBefore3rdTransfer
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.scoreBasket(),
                                new SleepAction(TIME.sleepAfter2ndUnclamp),
                                RobotActions.autoRetractAfterScoreBasket()
                        ),
                        RobotActions.setExtendo(MISC.intake3rdExtendoAngle, 0),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
//                        RobotActions.setRollers(0.8, TIME.waitBefore3rdTransfer),
                        RobotActions.runRollersUntilCollected(0.8, ColorRangefinderEx.SampleColor.YELLOW, TIME.waitBefore3rdTransfer, Extendo.LINKAGE_MAX_ANGLE)
                ))
                // Transfer, Setup, & Scores 3rd Objectively + Extend for 4th Objectively TODO sleep4thBeforeExtending
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.retractTransferAndSetupBasket(),
                                new SleepAction(TIME.sleepBefore3rdUnclamp),
                                RobotActions.scoreBasket()
                        ),
                        new SequentialAction(
                                new SleepAction(TIME.sleep4thBeforeExtending),
                                RobotActions.setExtendo(MISC.setupIntake4thExtendoAngle, 0)
                        )
                ))
                // Retract after 3rd Scored Objectively
                .afterTime(TIME.waitBefore3rdRetract, RobotActions.autoRetractAfterScoreBasket())

                // Moving to & Intaking 4th sample Objectively
                .afterTime(TIME.intake4thDelayAfter3rd, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(MISC.intake4thExtendoAngle, 0),
                        RobotActions.setRollers(0.8, 0)
                ))
//
                .strafeToLinearHeading(new Vector2d(POS.xIntakeSample3, POS.yIntakeSample3), Math.toRadians(HEAD.intake3rdSampleAngle))
//                .waitSeconds(TIME.waitBefore4thTransfer)
                .stopAndAdd(RobotActions.runRollersUntilCollected(0.8, ColorRangefinderEx.SampleColor.YELLOW, TIME.waitBefore4thTransfer, Extendo.LINKAGE_MAX_ANGLE))
                //Transfer while moving to Score 4th Sample Objectively
                .afterTime(0, RobotActions.retractTransferAndSetupBasket())
                .strafeToLinearHeading(new Vector2d(POS.xBasket3, POS.yBasket3), Math.toRadians(HEAD.dropOff4thSample))
                //Scoring 4th Sample Objectively
                .waitSeconds(TIME.sleepBefore4thUnclamp)
                .stopAndAdd(new SequentialAction(
                        RobotActions.scoreBasket(),
                        RobotActions.autoRetractAfterScoreBasket()
                ));
        return builder;
    }

    private TrajectoryActionBuilder scoreAllSubSamples(TrajectoryActionBuilder builder) {
        builder = builder.afterTime(0, new InstantAction(() -> autoAlignToSample.activateLimelight(8, ColorRangefinderEx.SampleColor.YELLOW)));

        builder = scoreSubSamples(builder, POS.sample5thOffset);
        builder = scoreSubSamples(builder, POS.sample6thOffset);
        builder = scoreSubSamples(builder, POS.sample7thOffset);
        builder = scoreSubSamples(builder, POS.sample8thOffset);
        builder = builder.afterTime(0, RobotActions.retractAfterScoreBasket());
        return builder;

    }

    private TrajectoryActionBuilder scoreSubSamples(TrajectoryActionBuilder builder, double offsetY) {
        builder = builder
                // Retract while moving to Sub from Scoring
                .afterTime(0.5, RobotActions.autoRetractAfterScoreBasket())
                .setTangent(Math.toRadians(HEAD.lineToSubTangent))
                .lineToY(POS.midSubY, ((pose2dDual, posePath, v) -> MISC.subVelocityConstraint), new ProfileAccelConstraint(MISC.subMinAccelConstraint, MISC.subMaxAccelConstraint))
                .splineToSplineHeading(new Pose2d(POS.subX, POS.subY + offsetY, Math.toRadians(0)), Math.toRadians(0), (pose2dDual, posePath, v) -> MISC.intakeSubVelocityConstraint)
                // Sweeping + Intaking Sub Sample TODO vision + remove hardcoded waits
                .stopAndAdd(new SequentialAction(
                        autoAlignToSample.detectTarget(TIME.secondsToExpire, false),
                        new InstantAction(() -> robot.limelightEx.enableStagelite(false))
                ))

                .stopAndAdd(new SequentialAction(
                        new InstantAction(autoAlignToSample::generateTargetTrajectory),
                        telemetryPacket -> {
                            robot.run();
                            return autoAlignToSample.getTargetSampleTrajectory().run(telemetryPacket);
                        }
                ))

                .setTangent(Math.toRadians(180))
                // Score Sub Sample
                .afterTime(0, RobotActions.retractTransferAndSetupBasket())
                .splineTo(new Vector2d(POS.midSubX, POS.midSubY), Math.toRadians(HEAD.lineToBasketTangent))
                .lineToY(POS.yBasketSub, ((pose2dDual, posePath, v) -> MISC.subVelocityConstraint), new ProfileAccelConstraint(MISC.subMinAccelConstraint, MISC.subMaxAccelConstraint))
                // Transfer & Setup after Intaking Sub Sample while Moving to Basket
                .stopAndAdd(new SequentialAction(
                        new SleepAction(TIME.waitBeforeUnclampSubBasket),
                        RobotActions.scoreBasket()
                ))
                .waitSeconds(TIME.sleepUntilLiftRetracted);
        return builder;
    }



}
