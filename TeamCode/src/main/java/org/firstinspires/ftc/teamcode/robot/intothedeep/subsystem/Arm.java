package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MAX_2;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Arm {
    private final ServoEx wrist, armstendo;

    private final ServoEx[] armServos;

    public static double
            NEUTRAL_ARM_ANGLE = 171, //215,
            TRANSFERRED_WRIST_ANGLE = 83,
            COLLECTING_ARM_ANGLE = 174, //240,
            COLLECTING_WRIST_ANGLE = 75, //140,
            BASKET_ARM_ANGLE = 10,
            BASKET_WRIST_ANGLE = 15,

            HANG_ARM_ANGLE = 60,

            WALL_PICKUP_ARM_ANGLE = 298,
            WALL_PICKUP_WRIST_ANGLE = 55,

            AUTON_TRANSFER_ARM_ANGLE = 100,

            GRAB_OFF_WALL_WRIST_ANGLE = 134,

            SCORE_SPECIMEN_ARM_ANGLE = 119,
            SCORE_SPECIMEN_WRIST_ANGLE = 81,
            RETRACT_SCORE_SPECIMEN_WRIST_ANGLE = 44,

            WALL_PICKUP_ARMSTENDO_ANGLE = 105,
            EXTENDED_ARMSTENDO_ANGLE = 10,
            TRANSFER_ARMSTENDO_ANGLE = 90, //50,
            RETRACTED_ARMSTENDO_ANGLE = 122;

    public enum WristAngle {
        COLLECTING,
        WALL_PICKUP,
        GRAB_OFF_WALL,
        RETRACT_SCORE_SPECIMEN,
        SCORE_SPECIMEN,
        TRANSFERRED,
        BASKET;

        public double getAngle() {
            switch (this) {
                case BASKET:                        return BASKET_WRIST_ANGLE;
                case WALL_PICKUP:                   return WALL_PICKUP_WRIST_ANGLE;
                case GRAB_OFF_WALL:                 return GRAB_OFF_WALL_WRIST_ANGLE;
                case SCORE_SPECIMEN:                return SCORE_SPECIMEN_WRIST_ANGLE;
                case RETRACT_SCORE_SPECIMEN:        return RETRACT_SCORE_SPECIMEN_WRIST_ANGLE;
                case TRANSFERRED:                   return TRANSFERRED_WRIST_ANGLE;
                case COLLECTING: default:           return COLLECTING_WRIST_ANGLE;
            }
        }
    }

    public enum ArmAngle {
        NEUTRAL,
        WALL_PICKUP,
        HANG,
        SCORE_SPECIMEN,
        COLLECTING,
        AUTON_TRANSFER,
        BASKET;

        public double getAngle() {
            switch (this) {
                case BASKET:                    return BASKET_ARM_ANGLE;
                case AUTON_TRANSFER:            return AUTON_TRANSFER_ARM_ANGLE;
                case HANG:                      return HANG_ARM_ANGLE;
                case WALL_PICKUP:               return WALL_PICKUP_ARM_ANGLE;
                case SCORE_SPECIMEN:            return SCORE_SPECIMEN_ARM_ANGLE;
                case COLLECTING:                return COLLECTING_ARM_ANGLE;
                case NEUTRAL: default:          return NEUTRAL_ARM_ANGLE;
            }
        }


    }

    public enum Extension {
        RETRACTED,
        EXTENDED,
        WALL_PICKUP,
        TRANSFER;

        public double getAngle() {
            switch (this) {
                case EXTENDED:                  return EXTENDED_ARMSTENDO_ANGLE;
                case WALL_PICKUP:               return WALL_PICKUP_ARMSTENDO_ANGLE;
                case TRANSFER:                  return TRANSFER_ARMSTENDO_ANGLE;
                case RETRACTED: default:        return RETRACTED_ARMSTENDO_ANGLE;
            }
        }
    }


    private WristAngle targetWristAngle = WristAngle.COLLECTING;
    private ArmAngle targetArmAngle = ArmAngle.NEUTRAL;
    private Extension targetArmstendoExtension = Extension.RETRACTED;
    public boolean isLocked = false;

    public Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        armstendo = new SimpleServo(hardwareMap, "armstendo", SERVO_AXON_MIN, SERVO_AXON_MAX_2);
        armServos = new ServoEx[] {
                new SimpleServo(hardwareMap, "arm master", SERVO_AXON_MIN, SERVO_AXON_MAX_2),
                new SimpleServo(hardwareMap, "arm follower", SERVO_AXON_MIN, SERVO_AXON_MAX_2)
        };

        armServos[1].setInverted(true);
    }

    public boolean setArmAngle(ArmAngle angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetArmAngle = angle;

        return true;
    }

    public boolean setArmAngle(ArmAngle angle) {
        return setArmAngle(angle, false);
    }

    public boolean setArmstendoAngle(Extension extension, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetArmstendoExtension = extension;

        return true;
    }

    public boolean setArmstendoAngle(Extension extension) {
        return setArmstendoAngle(extension, false);
    }

    public boolean setWristAngle(WristAngle angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetWristAngle = angle;

        return true;
    }

    public boolean setWristAngle(WristAngle angle) {
        return setWristAngle(angle, false);
    }

    public ArmAngle getArmAngle() {
        return targetArmAngle;
    }

    public Extension getArmstendoAngle() {return targetArmstendoExtension;}

    public WristAngle getWristAngle() {
        return targetWristAngle;
    }

     public void run() {
        wrist.turnToAngle(getWristAngle().getAngle());

        armstendo.turnToAngle(getArmstendoAngle().getAngle());

         for (ServoEx servos : armServos) {
            servos.turnToAngle(getArmAngle().getAngle());
        }
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM STATE:", targetArmAngle.name());
        mTelemetry.addData("WRIST STATE:", targetWristAngle.name());
        mTelemetry.addData("ARMSTENDO STATE", targetArmstendoExtension.name());
    }
}
