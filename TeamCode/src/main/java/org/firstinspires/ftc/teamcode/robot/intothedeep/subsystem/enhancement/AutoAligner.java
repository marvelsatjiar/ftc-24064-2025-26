package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.sensor.DistanceSensorEx;

@Config
public class AutoAligner {
    private final DistanceSensorEx
            leftDistanceSensor,
            rightDistanceSensor;

    private final PIDController xyPIDController = new PIDController();
    private final PIDController headingPIDController = new PIDController();

    public static PIDGains xyPIDGains = new PIDGains(
            0.0275,
            0.0002,
            0.0006,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains headingPIDGains = new PIDGains(
            0.00825,
            0.0002,
            0.0005,
            Double.POSITIVE_INFINITY
    );

    public static double
            SUBMERSIBLE_TARGET = 4,
            CLIMB_TARGET = 2,
            SUBMERSIBLE_DIRECTION = 180,
            CLIMB_DIRECTION = 270;

    private static final State
            SUBMERSIBLE_STATE = new State(SUBMERSIBLE_TARGET),
            CLIMB_STATE = new State(CLIMB_TARGET),
            SUBMERSIBLE_HEADING_STATE = new State(SUBMERSIBLE_DIRECTION),
            CLIMB_HEADING_STATE = new State(CLIMB_DIRECTION);

    public enum TargetDistance {
        SUBMERSIBLE,
        CLIMB,
        INACTIVE;

        private State toDistance() {
            switch (this) {
                case SUBMERSIBLE:       return SUBMERSIBLE_STATE;
                case CLIMB:             return CLIMB_STATE;
                case INACTIVE: default: return null;
            }
        }

        private State toHeading() {
            switch (this) {
                case SUBMERSIBLE:       return SUBMERSIBLE_HEADING_STATE;
                case CLIMB:             return CLIMB_HEADING_STATE;
                case INACTIVE: default: return null;
            }
        }
    }

//    public static final double SENSOR_DISTANCE = 2.5625;

    private TargetDistance targetDistance = TargetDistance.INACTIVE;

    private State currentXYState = new State(0);
    private State currentHeadingState = new State(0);

    public AutoAligner(HardwareMap hardwareMap) {
        xyPIDController.setGains(xyPIDGains);
        headingPIDController.setGains(headingPIDGains);

        leftDistanceSensor = new DistanceSensorEx(hardwareMap.get(DistanceSensor.class, "left distance"));
        rightDistanceSensor = new DistanceSensorEx(hardwareMap.get(DistanceSensor.class, "right distance"));
    }

    public TargetDistance getTargetDistance() {
        return targetDistance;
    }

    public void setTargetDistance(TargetDistance distance) {
        targetDistance = distance;

        headingPIDController.setTarget(distance.toHeading());
        xyPIDController.setTarget(distance.toDistance());
    }

    private double getXYDistance() {
        return xyPIDController.calculate(currentXYState);
    }

    private double getHeadingDistance() {
        return headingPIDController.calculate(currentHeadingState);
    }

    public PoseVelocity2d run(double y) {
        xyPIDController.setGains(xyPIDGains);
        headingPIDController.setGains(headingPIDGains);

        double leftCalculatedDistance = leftDistanceSensor.calculateDistance();
        double rightCalculatedDistance = rightDistanceSensor.calculateDistance();

        double theta = Math.toDegrees(robot.drivetrain.headingOffset - robot.drivetrain.pose.heading.toDouble());
        if (theta < 0) theta += 360;
        currentXYState = new State((rightCalculatedDistance + leftCalculatedDistance) / 2);
        currentHeadingState = new State(theta);


        return new PoseVelocity2d(
                new Vector2d(
                        getXYDistance(),
                        y
                ),
                -getHeadingDistance()
        );
    }

    public void printTelemetry() {
        mTelemetry.addData("current xy", currentXYState.x);
        mTelemetry.addData("current heading", currentHeadingState.x);
        mTelemetry.addData("target xy", (targetDistance.toDistance() == null ? "no target" : targetDistance.toDistance().x));
        mTelemetry.addData("target heading", (targetDistance.toHeading() == null ? "no target" : targetDistance.toHeading().x));
    }

}
