package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class Sweeper {

    public static double
            RETRACTED_ANGLE = 170,
            ACTIVE_ANGLE = 80,
            AUTON_ACTIVE_ANGLE = 0;

    public enum SweeperAngles {
        RETRACTED,
        ACTIVE,
        AUTON_ACTIVE;

        public double getAngle() {
            switch (this) {
                case RETRACTED:                  return RETRACTED_ANGLE;
                case ACTIVE:                    return ACTIVE_ANGLE;
                case AUTON_ACTIVE: default:     return AUTON_ACTIVE_ANGLE;
            }
        }
    }

    private final SimpleServo sweeper;

    public boolean isLocked = false;

    private SweeperAngles targetAngle = SweeperAngles.RETRACTED;

    public Sweeper(HardwareMap hardwareMap) {
        sweeper = new SimpleServo(hardwareMap, "sweeper", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
    }

    public boolean setAngle(SweeperAngles angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;

        targetAngle = angle;
        return true;
    }

    public boolean setAngle(SweeperAngles angle) {
        return setAngle(angle, false);
    }

    public boolean toggleSweeper(boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetAngle = targetAngle.getAngle() == ACTIVE_ANGLE ? SweeperAngles.RETRACTED : SweeperAngles.ACTIVE;

        return true;
    }

    public boolean toggleSweeper() {
        return toggleSweeper(false);
    }

    public SweeperAngles getSweeperAngle() {
        return targetAngle;
    }

    public void run() {
        sweeper.turnToAngle(targetAngle.getAngle());
    }

}
