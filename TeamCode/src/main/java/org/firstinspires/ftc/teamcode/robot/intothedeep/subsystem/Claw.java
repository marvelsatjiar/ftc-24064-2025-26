package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public final class Claw {

    public static double
            SAMPLE_CLAMP_ANGLE = 9,
            SPECIMEN_CLAMP_ANGLE = 0,
            WALL_PICKUP_ANGLE = 100,
            DEPOSIT_ANGLE = 35;

    public enum ClawAngles {
        SAMPLE_CLAMPED,
        SPECIMEN_CLAMPED,
        WALL_PICKUP,
        DEPOSIT;

        public double getAngle() {
            switch (this) {
                case SAMPLE_CLAMPED:        return SAMPLE_CLAMP_ANGLE;
                case SPECIMEN_CLAMPED:     return SPECIMEN_CLAMP_ANGLE;
                case WALL_PICKUP:           return WALL_PICKUP_ANGLE;
                case DEPOSIT: default:      return DEPOSIT_ANGLE;
            }
        }
    }

    private final ServoEx claw;

    private ClawAngles targetAngle = ClawAngles.DEPOSIT;

    public boolean isLocked = false;

    public Claw(HardwareMap hardwareMap) {
        claw = new SimpleServo(hardwareMap, "claw", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
    }

    public boolean setAngle(ClawAngles angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;

        targetAngle = angle;
        return true;
    }

    public boolean setAngle(ClawAngles angle) {
        return setAngle(angle, false);
    }

    public ClawAngles getClawAngle() {
        return targetAngle;
    }

    public void run() {
        claw.turnToAngle(targetAngle.getAngle());
    }
}

