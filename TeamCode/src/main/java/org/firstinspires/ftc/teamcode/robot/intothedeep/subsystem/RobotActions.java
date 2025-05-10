package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;

@Config
public class RobotActions {
    // 1. Add classes 2. delete unecessary delays 3. refactor delays
    public static class Transfer {
        public double
                setArmstendoTransferWait = 0,
                clampClawWait = 0.1,
                setCollectingArmWait = 0.2;
    }

    public static class RetractionForTransfer {
        public double
                retractLiftWait = 1,
                setNeutralArmWait = 0.4,
                retractExtendoWait = 0.7,
                setV4BUpWait = 0.2;
    }

    public static class ExtendIntake {
        public double extendExtendoWait = 0.1;
    }

    public static class SetupBasket {
        public double
                extendLiftFromWallPickupWait = 0.4,
                setArmstendoExtendedWait = 0.1,
                setArmstendoRetractedWait = 0.15,
                extendLiftToSetupWait = 0.3,
                setArmWait = 0.2,
                setClawWait = 0.1,
                sleepBeforeSetupAfterTransfer = 0.1,
                setV4BWait = 0;
    }

    public static class ScoreBasket {
        public double
                unclampClawToScoreWait = 0,
                retractArmstendoWait = 0,
                retractAutonArmWait = 0.2,
                retractArmWait = 0.5;


    }

    public static class DropSamples {
        public double
                setArmToDropSampleWait = 0.3,
                setWristWait = 0.2,
                unclampClawForDropoffWait = 0.3,
                setV4BWait = 0,

                armTransferWait = 0.15 ,


                retractToNeutralDelay = 0.4;
    }
    public static class InterleaveDrop {
        public double
                unclampClawForDropOffWait = 0.1,
                setClawWait = 0.1,
                retractLiftWait = 0,
                setArmstendoWait = 0.3,
                retractArmstendoWait = 0,
                setWristWait = 0.2,
                setArmWait = 0.2;
    }

    public static class LevelTwoHang {
        public double
                extendLiftForClimbWait = 1,
                setArmNeutralWait = 1,
                retractLiftWait = 3;
    }

    public static class SetupWallPickup {
        public double
                setLiftWait = 0.4,
                setWristWait = 0.15,
                retractArmstendoWait = 0.3,
                setArmstendoWait = 0.3,
                setArmWait = 0.2;
    }

    public static class SetupSpecimen {
        public double
                setArmForFirstSpecWait = 0.7,
                clampClawWait = 0.2,
                setWristWait = 0.2,
                setArmWait = 0.3,
                setArmstendoWait = 0.2,
                extendArmstendoWait = 0.3;
    }

    public static class ScoreSpecimen {
        public double
                retractArmstendoWait = 0.3,
                unclampClawWait = 0.2;
    }

    public static class SetupSpecimenStable {
        public double setArmWait = 0.2;
    }

    public static Transfer TRANSFER = new Transfer();
    public static RetractionForTransfer RETRACTION_FOR_TRANSFER = new RetractionForTransfer();
    public static ExtendIntake EXTEND_INTAKE = new ExtendIntake();
    public static SetupBasket SETUP_BASKET = new SetupBasket();
    public static ScoreBasket SCORE_BASKET = new ScoreBasket();
    public static DropSamples DROP_SAMPLES = new DropSamples();
    public static InterleaveDrop INTERLEAVE_DROP = new InterleaveDrop();
    public static LevelTwoHang LEVEL_TWO_HANG = new LevelTwoHang();
    public static SetupWallPickup SETUP_WALL_PICKUP = new SetupWallPickup();
    public static SetupSpecimen SETUP_SPECIMEN = new SetupSpecimen();
    public static ScoreSpecimen SCORE_SPECIMEN = new ScoreSpecimen();
    public static SetupSpecimenStable SETUP_SPECIMEN_STABLE = new SetupSpecimenStable();


    // DONE
    public static Action extendIntake(Extendo.Extension extension) {
        return new SequentialAction(
                setV4B(Intake.V4BAngle.UP, 0),
                setExtendo(extension, EXTEND_INTAKE.extendExtendoWait),
                new InstantAction(() -> robot.currentState = Robot.State.EXTENDO_OUT)
        );
    }




    // DONE
    public static Action retractForTransfer() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.setV4BUpWait),
                                        setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.retractExtendoWait)
                                ),
                                new SequentialAction(
                                        setArm(Arm.ArmAngle.NEUTRAL, RETRACTION_FOR_TRANSFER.setNeutralArmWait),
                                        setLift(Lift.Ticks.RETRACTED, RETRACTION_FOR_TRANSFER.retractLiftWait)
                                ),
                                setClaw(Claw.ClawAngles.DEPOSIT, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.TO_BE_TRANSFERRED)
                )
        );
    }

    // DONE
    public static Action retractTransferAndSetupBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        transfer(),
                        new SleepAction(SETUP_BASKET.sleepBeforeSetupAfterTransfer),
                        setupBasket(true)
                )
        );
    }

    // TODO
    public static Action transfer() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.TRANSFERRED,
                new SequentialAction(
                        new ParallelAction(
                                retractForTransfer(),
                                setWrist(Arm.WristAngle.COLLECTING, 0),
                                setV4B(Intake.V4BAngle.BEFORE_TRANSFER, 0),
                                setRollers(0.5, 0)
                        ),
                        setArmstendo(Arm.Extension.TRANSFER, TRANSFER.setArmstendoTransferWait),
                        setArm(Arm.ArmAngle.COLLECTING, TRANSFER.setCollectingArmWait),
                        setClaw(Claw.ClawAngles.SAMPLE_CLAMPED, TRANSFER.clampClawWait),
                        setRollers(0, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
                )
        );
    }

    // DONE
    public static Action setupBasket(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        setV4B(Intake.V4BAngle.TRANSFER, SETUP_BASKET.setV4BWait),
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_BASKET.setArmstendoRetractedWait),
                        setArm(Arm.ArmAngle.BASKET, SETUP_BASKET.setArmWait),
                        setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET, SETUP_BASKET.extendLiftToSetupWait),
                        setArmstendo(Arm.Extension.EXTENDED,SETUP_BASKET.setArmstendoExtendedWait),
                        setWrist(Arm.WristAngle.BASKET, 0),
                        setV4B(Intake.V4BAngle.UP, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
                )
        );
    }

    public static Action setupWithoutV4BBasket(boolean isHighBasket) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SCORE_BASKET,
                new SequentialAction(
                        setArm(Arm.ArmAngle.BASKET, SETUP_BASKET.setArmWait),
                        setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, SETUP_BASKET.setClawWait),
                        setLift(isHighBasket ? Lift.Ticks.HIGH_BASKET : Lift.Ticks.LOW_BASKET, SETUP_BASKET.extendLiftFromWallPickupWait),
                        setArmstendo(Arm.Extension.EXTENDED,SETUP_BASKET.setArmstendoExtendedWait),
                        setWrist(Arm.WristAngle.BASKET, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SCORE_BASKET)
                )
        );
    }

    // DONE
    public static Action scoreBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SCORED_BASKET,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.DEPOSIT, SCORE_BASKET.unclampClawToScoreWait),
                        new InstantAction(() -> robot.currentState = Robot.State.SCORED_BASKET)
                )
        );
    }

    public static Action retractAfterScoreBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        RobotActions.setArmstendo(Arm.Extension.RETRACTED,SCORE_BASKET.retractArmstendoWait),
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, SCORE_BASKET.retractArmWait),
                        RobotActions.retractToNeutral(0)
                )
        );
    }

    public static Action autoRetractAfterScoreBasket() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        RobotActions.setArmstendo(Arm.Extension.RETRACTED,SCORE_BASKET.retractArmstendoWait),
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, SCORE_BASKET.retractAutonArmWait),
                        RobotActions.retractToNeutral(0)
                )
        );
    }



    // TODO
    public static Action setupWallPickup() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.WALL_PICKUP,
                new SequentialAction(
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_WALL_PICKUP.retractArmstendoWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, SETUP_WALL_PICKUP.setWristWait),
                        setArm(Arm.ArmAngle.WALL_PICKUP, SETUP_WALL_PICKUP.setArmWait),
                        new ParallelAction(
                                setClaw(Claw.ClawAngles.WALL_PICKUP, 0),
                                setWrist(Arm.WristAngle.WALL_PICKUP, 0),
                                setV4B(Intake.V4BAngle.UP, 0),
                                setArmstendo(Arm.Extension.WALL_PICKUP, SETUP_WALL_PICKUP.setArmstendoWait)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.WALL_PICKUP)
                )
        );
    }
    public static Action wallPickupToNeutral() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_WALL_PICKUP.retractArmstendoWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, SETUP_WALL_PICKUP.setWristWait),
                        setArm(Arm.ArmAngle.NEUTRAL, SETUP_WALL_PICKUP.setArmWait),
                        new ParallelAction(
                                setClaw(Claw.ClawAngles.DEPOSIT, 0),
                                setWrist(Arm.WristAngle.COLLECTING, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }


    // TODO
    public static Action setupSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SPECIMEN,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, SETUP_SPECIMEN.clampClawWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, SETUP_SPECIMEN.setWristWait),
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_SPECIMEN.setArmstendoWait),

                        new ParallelAction(
                                setLift(Lift.Ticks.SETUP_SPECIMEN, 0),
                                setArm(Arm.ArmAngle.SCORE_SPECIMEN, SETUP_SPECIMEN.setArmWait),
                                setV4B(Intake.V4BAngle.UP, 0)
                        ),
                        new ParallelAction(
                                setWrist(Arm.WristAngle.SCORE_SPECIMEN, 0),
                                setArmstendo(Arm.Extension.EXTENDED, SETUP_SPECIMEN.extendArmstendoWait)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SPECIMEN)
                )
        );
    }

    public static Action setupFirstAutoSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_SPECIMEN,
                new SequentialAction(
                        setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, SETUP_SPECIMEN.clampClawWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, SETUP_SPECIMEN.setWristWait),
                        setArmstendo(Arm.Extension.RETRACTED, SETUP_SPECIMEN.setArmstendoWait),

                        new ParallelAction(
                                setLift(Lift.Ticks.AUTON_SETUP_FIRST_SPECIMEN, 0),
                                setArm(Arm.ArmAngle.SCORE_SPECIMEN, SETUP_SPECIMEN.setArmForFirstSpecWait),
                                setV4B(Intake.V4BAngle.UP, 0)
                        ),
                        new ParallelAction(
                                setWrist(Arm.WristAngle.SCORE_SPECIMEN, 0),
                                setArmstendo(Arm.Extension.EXTENDED, SETUP_SPECIMEN.extendArmstendoWait)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_SPECIMEN)
                )
        );
    }

    // TODO
    public static Action scoreSpecimen() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        new ParallelAction(
                                setClaw(Claw.ClawAngles.DEPOSIT, SCORE_SPECIMEN.unclampClawWait),
                                setWrist(Arm.WristAngle.RETRACT_SCORE_SPECIMEN, 0)
                        ),
                        retractAfterScore()
                )
        );
    }

    public static Action retractAfterScore() {
        return  new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new SequentialAction(
                        setArmstendo(Arm.Extension.RETRACTED, SCORE_SPECIMEN.retractArmstendoWait),
                        retractToNeutral(0),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
                )
        );
    }

    // DONE
    public static Action retractToNeutral(double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.NEUTRAL,
                new ParallelAction(
                        setArmstendo(Arm.Extension.RETRACTED, sleepSeconds),
                        setClaw(Claw.ClawAngles.DEPOSIT, 0),
                        setArm(Arm.ArmAngle.NEUTRAL, 0),
                        setWrist(Arm.WristAngle.COLLECTING, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL),
                        setLift(Lift.Ticks.RETRACTED, 0)
                )
        );
    }

    // DONE
    public static Action setupHang() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_HANG,
                new SequentialAction(
                        new ParallelAction(
                                setLift(Lift.Ticks.HANG_SETUP, LEVEL_TWO_HANG.extendLiftForClimbWait),
                                setWrist(Arm.WristAngle.RETRACT_SCORE_SPECIMEN, 0),
                                setArmstendo(Arm.Extension.RETRACTED, 0),
                                setV4B(Intake.V4BAngle.VERTICAL, 0),
                                setArm(Arm.ArmAngle.HANG, 0)
                        ),
                        new InstantAction(() -> robot.currentState = Robot.State.SETUP_HANG)
                )
        );
    }

    // DONE
    public static Action doHang() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.DO_HANG,
                new SequentialAction(
                        setLift(Lift.Ticks.HANG, LEVEL_TWO_HANG.retractLiftWait),
                        new InstantAction(() -> robot.currentState = Robot.State.DO_HANG)
                )
        );
    }

    // DONE
    public static Action setupDropSample() {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.SETUP_DROP_SAMPLE,
                new SequentialAction(
                        setV4B(Intake.V4BAngle.DOWN, DROP_SAMPLES.setV4BWait),
                        setArm(Arm.ArmAngle.BASKET, DROP_SAMPLES.setArmToDropSampleWait),
                        setWrist(Arm.WristAngle.BASKET, DROP_SAMPLES.setWristWait),
                        setClaw(Claw.ClawAngles.DEPOSIT, DROP_SAMPLES.unclampClawForDropoffWait),
                        retractToNeutral(0)
                )
        );
    }

    public static Action interleaveDropSample(double sleepSecondsBeforeDrop) {
        return new Actions.SingleCheckAction(
                () -> robot.currentState != Robot.State.WALL_PICKUP,
                new SequentialAction(
                        setV4B(Intake.V4BAngle.DOWN, DROP_SAMPLES.setV4BWait),
                        new ParallelAction(
                                setLift(Lift.Ticks.INTERLEAVE, 0),
                                setArm(Arm.ArmAngle.AUTON_TRANSFER, DROP_SAMPLES.armTransferWait)
                        ),
                        setArmstendo(Arm.Extension.RETRACTED, INTERLEAVE_DROP.retractArmstendoWait),
                        setWrist(Arm.WristAngle.GRAB_OFF_WALL, INTERLEAVE_DROP.setWristWait),
                        setArm(Arm.ArmAngle.WALL_PICKUP, INTERLEAVE_DROP.setArmWait),
                        new ParallelAction(
                                setWrist(Arm.WristAngle.WALL_PICKUP, 0),
                                setArmstendo(Arm.Extension.WALL_PICKUP, INTERLEAVE_DROP.setArmstendoWait )
                        ),
                        setLift(Lift.Ticks.RETRACTED, INTERLEAVE_DROP.retractLiftWait),
                        new SleepAction(sleepSecondsBeforeDrop),
                        setClaw(Claw.ClawAngles.WALL_PICKUP, INTERLEAVE_DROP.setClawWait),
//                        setV4B(Intake.V4BAngle.UP, 0),
                        new InstantAction(() -> robot.currentState = Robot.State.WALL_PICKUP)
                )
        );
    }



    // DONE
    public static Action retractExtendo() {
        return new SequentialAction(
                setRollers(0.7 , 0),
                setV4B(Intake.V4BAngle.UP, RETRACTION_FOR_TRANSFER.setV4BUpWait),
                setExtendo(Extendo.Extension.RETRACTED, RETRACTION_FOR_TRANSFER.retractExtendoWait),
                setRollers(0 , 0),
                new InstantAction(() -> robot.currentState = Robot.State.NEUTRAL)
        );
    }

    public static Action runRollersUntilCollected(double rollerPower, ColorRangefinderEx.SampleColor targetColor, double expireTime, double extendoAngle) {
        ElapsedTime outtakeSampleTimer = new ElapsedTime();

        return new SequentialAction(
                new InstantAction(() -> robot.intake.targetSampleTimer.reset()),
                new InstantAction(() -> robot.intake.setEdgeCases(targetColor)),
                setRollers(rollerPower, 0),
                telemetryPacket -> {
                    if (robot.intake.isOppositeSample()) {
                        robot.intake.setRollerPower(-1);
                        outtakeSampleTimer.reset();
                    } else {
                        robot.intake.setRollerPower(rollerPower);
                    }
                    return robot.intake.getCurrentSample() != targetColor && robot.intake.targetSampleTimer.seconds() <= expireTime;
                }
        );
    }

/*

----------------------------------------------------------------------------------------------------

 */

    public static Action setV4B(Intake.V4BAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getTargetV4BAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.intake.setTargetV4BAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setExtendo(Extendo.Extension extension, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getTargetExtension() != extension,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetExtension(extension, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setExtendo(double angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.extendo.getTargetAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.extendo.setTargetAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setArmstendo(Arm.Extension extension, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmstendoAngle() != extension,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmstendoAngle(extension, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    private static Action setLift(Lift.Ticks ticks, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.lift.getTargetTicks() != ticks,
                new ParallelAction(
                        new InstantAction(() -> robot.lift.setTargetTicks(ticks, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setArm(Arm.ArmAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getArmAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setArmAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setWrist(Arm.WristAngle angle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.arm.getWristAngle() != angle,
                new ParallelAction(
                        new InstantAction(() -> robot.arm.setWristAngle(angle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setClaw(Claw.ClawAngles clawAngle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.claw.getClawAngle() != clawAngle,
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setAngle(clawAngle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }

    public static Action setSweeper(Sweeper.SweeperAngles sweeperAngle, double sleepSeconds) {
        return new Actions.SingleCheckAction(
                () -> robot.sweeper.getSweeperAngle() != sweeperAngle,
                new ParallelAction(
                        new InstantAction(() -> robot.sweeper.setAngle(sweeperAngle, true)),
                        new SleepAction(sleepSeconds)
                )
        );
    }


    public static Action setRollers(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.setRollerPower(power, true)),
                new SleepAction(sleepSeconds)
        );
    }
}


