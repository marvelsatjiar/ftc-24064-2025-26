package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;
import java.util.TreeMap;

@Config
public class AutoAlignToSample {
    private final LimelightEx limelightEx;

    public ColorRangefinderEx.SampleColor targetColor;

    private final TreeMap<Double, Double> extendoLUT = new TreeMap<>();

    private double
        txAfterCalc = 0,
        tYAfterCalc = 0,
        xDistanceAfterCalc = 0,
        yDistanceAfterCalc = 0;

    private boolean isDebugVariablesSet = false;

    public static double
            xOffset = 5.25,
            yOffset = 2,
            sampleOffset = 2.5,
            limelightTilt = 61,
            limelightHeight = 11,
            extendoOffset = 3;

    public static double secondsUntilCollected = 0.2;

    private double
            finalDistance = 0,
            desiredSampleX = 0,
            desiredSampleY = 0;

    private Action targetSampleTrajectory;
    
    public static class AutoAlign {
        public double
                delayBeforeRollers = 0.3,
                delayBeforeFullExtension = 0.6,
                sleepSecondsBeforeV4bUp = 0.4,
                sleepSecondsBeforeV4bDown = 0.1,
                sleepSecondsBeforeRollersDeactivate = 0.2;
    }

    private boolean
            isSampleDetected = false,
            isTurningOnly = false;

    private double
            offsetExtendoAngle = 0,
            xDistance = 0,
            yDistance = 0,
            xDistanceFromCenter = 0,
            yDistanceFromCenter = 0,
            headingAngle = 0,
            targetedExtendoAngle = 0;

    public static AutoAlign A_A = new AutoAlign();

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        // x = distance, y = extendo angle
        extendoLUT.put(9.0, 13.0);
        extendoLUT.put(10.0, 43.0);
        extendoLUT.put(11.0, 53.0);
        extendoLUT.put(12.0, 58.0);
        extendoLUT.put(13.0, 63.0);
        extendoLUT.put(14.0, 68.0);
        extendoLUT.put(15.0, 70.0);
        extendoLUT.put(16.0, 76.0);
        extendoLUT.put(17.0, 81.0);
        extendoLUT.put(18.0, 84.0);
        extendoLUT.put(19.0, 87.0);
        extendoLUT.put(20.0, 93.0);
        extendoLUT.put(21.0, 97.0);
        extendoLUT.put(22.0, 105.0);
        extendoLUT.put(23.0, 109.0);
        extendoLUT.put(24.0, 115.0);
        extendoLUT.put(25.0, 127.0);
        extendoLUT.put(25.5, 142.0);



        this.limelightEx = limelightEx;
    }

    public void activateLimelight(int detectionPipeline, ColorRangefinderEx.SampleColor color) {
        targetColor = color;

        limelightEx.enableStagelite(true);

        limelightEx.getLimelight().pipelineSwitch(detectionPipeline);

        limelightEx.getLimelight().setPollRateHz(10);
    }

    // add lock mechanism here (for sample)- for this you must also get current detections!!!
    public boolean targetSample() {
        limelightEx.update();
        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (targets != null && !targets.isEmpty() && targets.get(0) != null) {
            desiredSampleX = targets.get(0).getTargetXDegrees() + 0.0;
            desiredSampleY = targets.get(0).getTargetYDegrees() + 0.0;
            return true;
        }

        return false;
 }

    private double calculateExtendoTarget(double offsetDistance) {
                finalDistance = 0;
            if (isTurningOnly) finalDistance = Math.sqrt(yDistanceFromCenter*yDistanceFromCenter + xDistanceFromCenter*xDistanceFromCenter) - sampleOffset - 0.625;
            else finalDistance = (yDistanceFromCenter-sampleOffset);
            finalDistance += offsetDistance;
            finalDistance = Range.clip(finalDistance, extendoLUT.firstKey(), extendoLUT.lastKey());

            if (extendoLUT.containsKey(finalDistance)) return extendoLUT.get(finalDistance);

            if (finalDistance >= extendoLUT.firstKey() && finalDistance <= extendoLUT.lastKey()) {
                double x2 = extendoLUT.ceilingKey(finalDistance); // x2
                double x1 = extendoLUT.floorKey(finalDistance); // x1

                double y2 = extendoLUT.get(x2); // y2
                double y1 = extendoLUT.get(x1); // y1

                return y1 + ((finalDistance - x1) * (y2 - y1)) / (x2 - x1);
            }

            return robot.extendo.getTargetAngle();
    }

    private Pose2d calculateTargetPosition(boolean isTurning) {

        headingAngle = Math.atan2(xDistanceFromCenter, yDistanceFromCenter);

        if (isTurning) {
            isTurningOnly = true;
            return new Pose2d(0, 0, headingAngle);
        } else {
            return new Pose2d(0, xDistanceFromCenter, 0); // should be xDistance TODO
        }
    }

//    private ColorRangefinderEx.SampleColor getTargetColor() {
//        if (desiredSample != null) {
//            switch (desiredSample.getClassId()) {
//                case 0:
//                    return ColorRangefinderEx.SampleColor.BLUE;
//                case 1:
//                    return ColorRangefinderEx.SampleColor.RED;
//                case 2:
//                    return ColorRangefinderEx.SampleColor.YELLOW;
//            }
//        }
//        return ColorRangefinderEx.SampleColor.NOTHING;
//    }

    public Action detectTarget(double secondsToExpire, boolean isTurning) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    isSampleDetected = false;
                    isFirstTime = false;
//                    limelightEx.getLimelight().captureSnapshot("detection");
                    expirationTimer.reset();
                }

                if (!isSampleDetected) {
                    isSampleDetected = targetSample();
                    if (isSampleDetected) limelightEx.getLimelight().captureSnapshot("detected");

                }

                if (isSampleDetected) {

                    yDistance = Math.tan(Math.toRadians(limelightTilt + desiredSampleY)) * limelightHeight;
                    xDistance = Math.tan(Math.toRadians(desiredSampleX)) * yDistance;

                    if (!isDebugVariablesSet) {
                        isDebugVariablesSet = true;

                        txAfterCalc = desiredSampleX;
                        tYAfterCalc = desiredSampleY;
                        xDistanceAfterCalc = xDistance;
                        yDistanceAfterCalc = yDistance;
                    }

                    yDistanceFromCenter = yDistance + yOffset;
                    xDistanceFromCenter = xDistance + xOffset;
                    
                    // send out output to motors/servos
                    targetedPoseOffset = calculateTargetPosition(isTurning);
                    targetedExtendoAngle = calculateExtendoTarget(0);
                    offsetExtendoAngle = calculateExtendoTarget(extendoOffset) ;
//                  setEdgeCases();
                }

                mTelemetry.addData("is sample targeted? ", isSampleDetected);
                mTelemetry.addData("is expired? ", expirationTimer.seconds() > secondsToExpire);

                mTelemetry.update();

                return expirationTimer.seconds() <= secondsToExpire && !isSampleDetected;
            }
        };
    }

    public void generateTargetTrajectory() {
        if (isSampleDetected) {
            if (!isTurningOnly) {
                targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .afterTime(0, RobotActions.setExtendo(targetedExtendoAngle, 0))
                        .afterTime(A_A.sleepSecondsBeforeV4bDown, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                        .afterTime(A_A.delayBeforeRollers, RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected, offsetExtendoAngle))
                        .afterTime(A_A.delayBeforeFullExtension, RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0))
                        .strafeTo(new Vector2d(robot.drivetrain.pose.position.x + targetedPoseOffset.position.x, robot.drivetrain.pose.position.y + targetedPoseOffset.position.y))
                        .waitSeconds(secondsUntilCollected)

                        .build();
            } else {
                targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .afterTime(0, RobotActions.setExtendo(targetedExtendoAngle, 0))
                        .afterTime(A_A.sleepSecondsBeforeV4bDown, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                        .afterTime(A_A.delayBeforeRollers, RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected, offsetExtendoAngle))
                        .afterTime(A_A.delayBeforeFullExtension, RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0))
                        .turn(-targetedPoseOffset.heading.toDouble(), new TurnConstraints(4.5, -Math.PI, Math.PI))
                        .waitSeconds(secondsUntilCollected)
                        .build();
            }
        } else {
            targetSampleTrajectory = new NullAction();
        }
    }

    public Action setFullExtensionIfNotCollected() {
        if (!robot.intake.isCorrectSample()) {
            return new SequentialAction(
                    RobotActions.setExtendo(Extendo.Extension.EXTENDED, A_A.sleepSecondsBeforeV4bUp),
                    RobotActions.setV4B(Intake.V4BAngle.UP, A_A.sleepSecondsBeforeRollersDeactivate),
                    RobotActions.retractForTransfer()
            );
        } else {
            return RobotActions.retractForTransfer();
        }
    }

    public Action updateTelemetry(boolean isOpModeActive) {
        return new Actions.RunnableAction(
                () -> {
                    mTelemetry.addData("tx after calc : ", txAfterCalc);
                    mTelemetry.addData("ty after calc : ", tYAfterCalc);

                    mTelemetry.addData("heading (in radians) : ", headingAngle);

                    mTelemetry.addData("y distance after calc: ", yDistanceAfterCalc);
                    mTelemetry.addData("x distance after calc : ", xDistanceAfterCalc);

                    mTelemetry.addData("y distance : ", yDistance);
                    mTelemetry.addData("x distance : ", xDistance);

                    mTelemetry.addData("extendo distance : ", finalDistance);
                    mTelemetry.addData("extendo angle : ", targetedExtendoAngle);
                    return isOpModeActive;
                }
        );
    }

    public boolean wasSampleDetected() {
        return isSampleDetected;
    }

    public Action getTargetSampleTrajectory() {
        return targetSampleTrajectory;
    }
}
