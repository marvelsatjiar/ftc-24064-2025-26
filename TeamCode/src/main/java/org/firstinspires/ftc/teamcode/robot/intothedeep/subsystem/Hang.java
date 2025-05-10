package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Hang {

    private CRServo hangMaster, hangFollower;

    private double input1, input2;

    public Hang(HardwareMap hardwareMap) {
        hangMaster = hardwareMap.get(CRServo.class, "hang master");
        hangFollower = hardwareMap.get(CRServo.class, "hang follower");

        hangFollower.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power1, double power2) {
        input1 = power1;
        input2 = power2;
    }

    public void run() {
        hangMaster.setPower(input1);
        hangFollower.setPower(input2);
    }

}
