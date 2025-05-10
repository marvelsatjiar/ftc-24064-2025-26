package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public final class Common {
    public static Pose2d AUTO_END_POSE = null;
    public static Pose AUTO_END_POSE_PEDRO = null;

    public static boolean IS_RED = true, IS_SPECIMEN_SIDE = false;

    public static final double
            LEFT = Math.toRadians(180),
            FORWARD = Math.toRadians(90),
            RIGHT = Math.toRadians(0),
            BACKWARD = Math.toRadians(270),
            SERVO_25_KG_MIN = 0,
            SERVO_25_KG_MAX = 270,
            SERVO_45_KG_MIN = 0,
            SERVO_45_KG_MAX = 270,
            SERVO_AXON_MAX_1 = 270,
            SERVO_AXON_MIN = 0,
            SERVO_AXON_MAX_2 = 355;

    public static final int
            LIMELIGHT_RED_DETECTION_PIPELINE = 7,
            LIMELIGHT_BLUE_DETECTION_PIPELINE = 6 ,
            LIMELIGHT_SPECIMEN_NN_PIPELINE = 4;

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;
    public static MultipleTelemetry mTelemetry;

}
