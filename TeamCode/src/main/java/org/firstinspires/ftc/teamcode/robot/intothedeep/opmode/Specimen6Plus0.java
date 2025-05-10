package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
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
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(name = "Specimen 6+0")
@Config
public class Specimen6Plus0 extends AbstractAuto {
    private AutoAlignToSample autoAlignToSample;

    public boolean isSevenPlusZero = false;

    private boolean isConfirmed = false;

    private Action currentTraj = new NullAction();

    public static class GiveSamples {
        public double
                // Position
                intermediaryX = 32.5,
                intermediaryY = -51,

                intakeSampleX = 52,
                intakeSampleY = -41.5,

                outtakeSampleX = 52,
                outtakeSampleY = -41.5,


                // Headings
                tangentBeforeFirstSample = 0,
                tangentForIntermediaryPosition = 0,

                intakeFirstSampleHeading = 93,
                intakeSecondSampleHeading = 54,
                intakeThirdSampleHeading = 30,
                outtakeFirstSampleHeading = -30,
                outtakeSecondSampleHeading = -70,
                intakeFirstExtendoAngle = 80,
                intermediarySecondExtendoAngle = 50,
                intakeSecondExtendoAngle = 77,
                intakeThirdExtendoAngle = 130,
                outtakeExtendoAngleFirst = 70,
                outtakeExtendoAngleSecond = 80,

                // Timings
                waitBefore2ndIntake = 0.3,
                waitBeforeWallPickup = 1.2,
                sleepBeforeInterleave3rdSample = 0,
                sleepBeforeInterleaveSample = 0.5,
                firstIntakeDelay = 0.7,
                secondIntakeDelay = 0.2,
                secondIntermediaryDelay = 0,
                thirdIntakeDelay = 0.6,
                firstSleepBeforeTurning = 0.3,
                secondSleepBeforeTurning = 0.5,
                thirdSleepBeforeTransfer = 0.5,
                outtakeFirstSampleDelay = 0.6,
                outtakeSecondSampleDelay = 0.5,
                outtake1stTime = 0.3,
                outtake2ndTime = 0.3,
                firstOuttakeWait = 0.15,
                sleepBeforeV4B = 0,

                // Constraints
                intermediaryVelocityConstraint = 60,
                outtakeAngularVelocityConstraint = 4.5,
                // Roller Power
                intakeRollerPower = 1,
                outtakeRollerPower = -1;
    }

    public static class ScoreSpecimens {
        public double
                // Positions
                subSampleX = -1,
                scoreSpecimenY = -28.5,

                wallPickupX = 34,
                secondWallPickupX = 52,
                intakeSpecimenY = -68.5,
                intermediaryIntakeSecondSpecimenY = -47,
                intakeSecondSpecimenY = -64.5,
                intermediaryIntakeSpecimenY = -58.5,
                specimen2ndOffsetX = -12,
                specimen3rdOffsetX = -13,
                specimen4thOffsetX = -14,
                specimen5thOffsetX = -7,
                specimen6thOffsetX = -7,
                secondSpecimenOffsetY = 12,
                thirdSpecimenOffsetY = 9,
                fourthSpecimenOffsetY = 11,
                fifthSpecimenOffsetY = 11,
                sixthSpecimenOffsetY = 9,
                firstWallOffsetY = 2,
                secondWallOffsetY = 2,
                thirdWallOffsetY = 2,
                fourthWallOffsetY = 2,
                fifthWallOffsetY = 2,


                // Headings
                scoringAngle = 90,

                // Timings
                waitBeforeVision = 0.1,
                waitBeforeSetup2ndSpecimen = 0.1,
                intermediaryToClampSecond = 0.6,
                waitBeforeDriveOffVison = 0.3,
                waitBeforeMovingToIntermediary = 0.5,
                delayBeforeRFT = 0.2,
                secondsToExpire = 1.5,
                timeBeforeWallPickup = 1,
                timeBeforeWrist = 0.3,
                timeBeforeMoving = 0,
                lastFourSleepBeforeGrab = 0,
                secondSleepBeforeSetup = 0.1,
                sleepSecondsBeforeUnclampFirst = 1.3,
                sleepSecondsBeforeUnclampSecond = 2.3,
                sleepSecondsBeforeUnclampThird = 1.7,
                sleepSecondsBeforeUnclampFourth = 1.7,
                sleepSecondsBeforeUnclampFifth = 1.8,
                sleepSecondsBeforeUnclampSixth = 2,
                goofyLoopTimes = 0.1,

                //Constraints
                minWallPickupProfileAccel = -60,
                maxWallPickupProfileAccel = 60,
                wallPickUpVelocityConstraint = 30,
                scoreSpecimenVelocityConstraint = 90,
                minScoreProfileAccel = -140,
                maxScoreProfileAccel = 150;
    }

    public static GiveSamples G_S = new GiveSamples();
    public static ScoreSpecimens S_S = new ScoreSpecimens();


    public static double
            startingPositionX = 7.375,
            startingPositionY = -60,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2;

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            if (gamepadEx1.wasJustPressed(A)) isSevenPlusZero = !isSevenPlusZero;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) S_S.subSampleX--;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) S_S.subSampleX++;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine("| A - Toggle 6+0");
            mTelemetry.addLine("| DPAD LEFT - Subtract estimated 6th sample | DPAD RIGHT - Add estimated 6th sample");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addData("Six Plus Zero : ", isSevenPlusZero);
            mTelemetry.addLine("Estimated 6th sample is : " + S_S.subSampleX);
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();

            if (gamepadEx1.wasJustPressed(LEFT_BUMPER) && gamepadEx1.wasJustPressed(RIGHT_BUMPER)) {
                isConfirmed = true;
                autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE);

                TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

                builder = scoreFirstSpecimen(builder);
                builder = giveSamples(builder);
                builder = scoreAllSpecimens(builder);
//                builder = park(builder);

                currentTraj = builder.build();
            }
        }

        if (!isConfirmed) {
            autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE);

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

            builder = scoreFirstSpecimen(builder);
            builder = giveSamples(builder);
            builder = scoreAllSpecimens(builder);
//                builder = park(builder);

            currentTraj = builder.build();
        }
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(90));
    }

    @Override
    protected void onInit() {
        super.onInit();

        autoAlignToSample = new AutoAlignToSample(robot.limelightEx);

        robot.arm.setArmAngle(Arm.ArmAngle.WALL_PICKUP);
        robot.arm.setWristAngle(Arm.WristAngle.GRAB_OFF_WALL);
        robot.arm.setArmstendoAngle(Arm.Extension.RETRACTED);
        robot.claw.setAngle(Claw.ClawAngles.SPECIMEN_CLAMPED);
        robot.intake.setTargetV4BAngle(Intake.V4BAngle.VERTICAL);
        robot.extendo.setTargetExtension(Extendo.Extension.RETRACTED);

        robot.setCurrentState(Robot.State.WALL_PICKUP);

        robot.intake.run(robot.extendo.getTargetAngle());
        robot.extendo.run(false);
        robot.arm.run();
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        return currentTraj;
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(extendSleep, new ParallelAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0),
                        RobotActions.setArm(Arm.ArmAngle.BASKET, 0),
                        RobotActions.setWrist(Arm.WristAngle.BASKET, 0),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315));
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeUnclamp, double wallOffsetY) {
        // Scoring
        builder = builder
                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen());

        builder = builder.strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(S_S.scoringAngle), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel));

        if (!doPark)
            builder = builder.afterTime(S_S.timeBeforeWallPickup, RobotActions.setupWallPickup());


        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY + wallOffsetY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint, new ProfileAccelConstraint(S_S.minWallPickupProfileAccel, S_S.maxWallPickupProfileAccel));

        // Setting up for the next cycle
        if (!doPark) builder = builder
                .stopAndAdd(new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))
                .afterTime(S_S.lastFourSleepBeforeGrab, RobotActions.setupSpecimen());


        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(S_S.secondWallPickupX, S_S.intermediaryIntakeSecondSpecimenY), Math.toRadians(90))
                .afterTime(S_S.intermediaryToClampSecond, new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))
                .lineToY(S_S.intakeSecondSpecimenY)
                .waitSeconds(S_S.waitBeforeSetup2ndSpecimen)
                .afterTime(S_S.secondSleepBeforeSetup, RobotActions.setupSpecimen());

        builder = scoreSpecimen(builder, S_S.specimen2ndOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond, S_S.firstWallOffsetY);
        builder = scoreSpecimen(builder, S_S.specimen3rdOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird, S_S.secondWallOffsetY);
        builder = scoreSpecimen(builder, S_S.specimen4thOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth, S_S.thirdWallOffsetY);
        builder = scoreSpecimen(builder, S_S.specimen5thOffsetX, S_S.fifthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFifth, S_S.fourthWallOffsetY);
        builder = scoreSpecimen(builder, S_S.specimen6thOffsetX, S_S.sixthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampSixth, S_S.fifthWallOffsetY);

        return builder;
    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(325))
                .afterTime(G_S.sleepBeforeInterleaveSample, new SequentialAction(
                        RobotActions.transfer(),
                        RobotActions.interleaveDropSample(0)))
                .splineToSplineHeading(new Pose2d(G_S.intermediaryX, G_S.intermediaryY, Math.toRadians(90)), Math.toRadians(G_S.tangentForIntermediaryPosition), (pose2dDual, posePath, v) -> G_S.intermediaryVelocityConstraint)

                // Intaking 1st
                .afterTime(G_S.firstIntakeDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.wallPickupToNeutral(),
                        RobotActions.setExtendo(G_S.intakeFirstExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))
                .setTangent(Math.toRadians(G_S.tangentBeforeFirstSample))
                .splineToSplineHeading(new Pose2d(G_S.intakeSampleX, G_S.intakeSampleY, Math.toRadians(G_S.intakeFirstSampleHeading)), Math.toRadians(35), (pose2dDual, posePath, v) -> G_S.intermediaryVelocityConstraint)
                .waitSeconds(G_S.firstSleepBeforeTurning)

                // Outtaking 1st
                .afterTime(0, RobotActions.setExtendo(G_S.outtakeExtendoAngleFirst, 0))
                .afterTime(0, new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.BEFORE_TRANSFER, G_S.outtakeFirstSampleDelay),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.outtake1stTime),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))
                //.afterTime(G_S.setV4bDownWhenFirstSampleOuttakeDelay, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeFirstSampleHeading), new AngularVelConstraint(G_S.outtakeAngularVelocityConstraint))
                // Intaking 2nd
                .afterTime(G_S.secondIntermediaryDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.intermediarySecondExtendoAngle, 0)
                ))
                .waitSeconds(G_S.waitBefore2ndIntake)
                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeSecondSampleHeading))
                .afterTime(G_S.secondIntakeDelay, RobotActions.setExtendo(G_S.intakeSecondExtendoAngle, 0))
                .waitSeconds(G_S.secondSleepBeforeTurning)

                // Outtaking 2nd
                .afterTime(0, RobotActions.setExtendo(G_S.outtakeExtendoAngleSecond, 0))
                .afterTime(G_S.outtakeSecondSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.BEFORE_TRANSFER, G_S.outtake2ndTime),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))

                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeSecondSampleHeading), new AngularVelConstraint(G_S.outtakeAngularVelocityConstraint))

                //Intaking 3rd
                .afterTime(G_S.thirdIntakeDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.intakeThirdExtendoAngle, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeThirdSampleHeading))
                .waitSeconds(G_S.thirdSleepBeforeTransfer)
                //Outtaking 3rd
//                .afterTime(0, RobotActions.transfer())
//                .afterTime(G_S.outtakeThirdSampleDelay, new SequentialAction(
//                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.thirdSleepBeforeV4B),
//                        RobotActions.setV4B(Intake.V4BAngle.BEFORE_TRANSFER, 0)
//                ))
//                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeThirdSampleHeading), new AngularVelConstraint(G_S.outtakeAngularVelocityConstraint))
                .afterTime(0, new SequentialAction(
                        RobotActions.transfer(),
                        RobotActions.interleaveDropSample(G_S.sleepBeforeInterleave3rdSample)))
                .waitSeconds(G_S.waitBeforeWallPickup);

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.setupFirstAutoSpecimen())
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP, 0))
                .afterTime(0, autoAlignToSample.updateTelemetry(!opModeIsActive()))
                .waitSeconds(S_S.goofyLoopTimes)
                .afterTime(S_S.sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
                .strafeToConstantHeading(new Vector2d(S_S.subSampleX, S_S.scoreSpecimenY)) //, (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));
                .waitSeconds(S_S.waitBeforeVision)
                .stopAndAdd(autoAlignToSample.detectTarget(S_S.secondsToExpire, true))
                .waitSeconds(S_S.waitBeforeDriveOffVison)
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(autoAlignToSample::generateTargetTrajectory),
                                telemetryPacket -> {
                                    robot.run();
                                    return autoAlignToSample.getTargetSampleTrajectory().run(telemetryPacket);
                                }
                        )
                )
                .afterTime(S_S.delayBeforeRFT, RobotActions.retractForTransfer())
                .waitSeconds(S_S.waitBeforeMovingToIntermediary);
        // transfer

        return builder;
    }
}
