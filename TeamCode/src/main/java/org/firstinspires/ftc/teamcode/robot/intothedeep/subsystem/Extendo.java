package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MAX_1;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MAX_2;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public final class Extendo {
    // Constants and classes made to make the subssystem work properly
    SimpleServo[] linkageServos;

    public static double
            LINKAGE_MIN_ANGLE = 13,
            LINKAGE_MAX_ANGLE = 142,
            LINKAGE_ONE_FOURTH_ANGLE = (LINKAGE_MAX_ANGLE - LINKAGE_MIN_ANGLE) * (1/4.0),
            LINKAGE_ONE_HALF_ANGLE = (LINKAGE_MAX_ANGLE - LINKAGE_MIN_ANGLE) * (1/2.0),
            LINKAGE_THREE_FOURTHS_ANGLE = (LINKAGE_MAX_ANGLE - LINKAGE_MIN_ANGLE) * (3/4.0),
            STICK_MULT = 4;

    public static double
            LINKAGE_LENGTH_A = 10,
            LINKAGE_LENGTH_B = 11,
            PIVOT_HEIGHT = 1.25;

    public enum Extension {
        RETRACTED,
        ONE_FOURTH,
        ONE_HALF,
        THREE_FOURTHS,
        CUSTOM_EXTENSION,
        EXTENDED;

        public double getAngle() {
            switch (this) {
                case EXTENDED: return LINKAGE_MAX_ANGLE;
                case ONE_FOURTH: return LINKAGE_ONE_FOURTH_ANGLE;
                case ONE_HALF: return LINKAGE_ONE_HALF_ANGLE;
                case THREE_FOURTHS: return LINKAGE_THREE_FOURTHS_ANGLE;
                case RETRACTED: default: return LINKAGE_MIN_ANGLE;
            }
        }
    }

    private double targetAngle = LINKAGE_MIN_ANGLE;
    private Extension targetExtension = Extension.RETRACTED;

    public boolean isLocked = false;

    /**
     * Instantiates a new 'extendo' class that holds the extendo mechanisms (methods and functions)
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Extendo(HardwareMap hardwareMap) {
        SimpleServo masterLinkage = new SimpleServo(hardwareMap, "extendoLinkageMain", SERVO_AXON_MIN, SERVO_AXON_MAX_2);
        SimpleServo followerLinkage = new SimpleServo(hardwareMap, "extendoLinkageFollower", SERVO_AXON_MIN, SERVO_AXON_MAX_2);

        masterLinkage.setInverted(true);

        linkageServos = new SimpleServo[]{masterLinkage, followerLinkage};
    }

    // Returns the state of the extendo
    public Extension getTargetExtension() {
        return targetExtension;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean setTargetExtension(Extension extension, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetExtension = extension;
        targetAngle = extension.getAngle();

        return true;
    }

    public boolean setTargetAngle(double angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetAngle = Range.clip(angle, LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);
        if (targetAngle == LINKAGE_MIN_ANGLE) targetExtension = Extension.RETRACTED;
        else targetExtension = Extension.CUSTOM_EXTENSION;

        return true;
    }

    public void setTargetAngleWithStick(double stick) {
        setTargetAngle(targetAngle + stick * STICK_MULT, false);
    }

    public boolean setTargetExtension(Extension extension) {
        return setTargetExtension(extension, false);
    }

    // Runs each servo inside of the group to a certain angle base on what was given
    public boolean run(boolean isV4BUnsafe) {
        if (isV4BUnsafe) return false;
        for (SimpleServo servos : linkageServos) {
            servos.turnToAngle(targetAngle);
        }

        return true;
    }

    public double convertTargetInchesToExtensionAngle(double targetInches) {
        targetInches = Range.clip(targetInches, 3, 20);

        double c = (LINKAGE_LENGTH_B * LINKAGE_LENGTH_B - LINKAGE_LENGTH_A * LINKAGE_LENGTH_A - PIVOT_HEIGHT * PIVOT_HEIGHT - targetInches * targetInches);
        double baseSquared = (-2 * LINKAGE_LENGTH_A) * Math.sqrt(PIVOT_HEIGHT * PIVOT_HEIGHT + targetInches * targetInches);

        return Range.clip(Math.toDegrees(Math.asin(c / baseSquared) + Math.atan2(targetInches, PIVOT_HEIGHT)) - 77, LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        mTelemetry.addData("Extendo is", targetExtension);
        mTelemetry.addData("Servo angle is", targetAngle);
    }
}
