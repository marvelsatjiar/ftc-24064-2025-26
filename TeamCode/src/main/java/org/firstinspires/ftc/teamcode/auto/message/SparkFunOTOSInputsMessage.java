package org.firstinspires.ftc.teamcode.auto.message;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public final class SparkFunOTOSInputsMessage {
    public long timestamp;
    public SparkFunOTOS.Pose2D pos;
    public SparkFunOTOS.Pose2D vel;

    public SparkFunOTOSInputsMessage(SparkFunOTOS.Pose2D pos, SparkFunOTOS.Pose2D vel) {
        this.timestamp = System.nanoTime();
        this.pos = pos;
        this.vel = vel;
    }
}
