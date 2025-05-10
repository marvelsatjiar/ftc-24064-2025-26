package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class CachedMotor extends MotorEx {
    private double currentOutput;
    
    public static double roundingPoint = 1000;

    public CachedMotor(@NonNull HardwareMap hardwareMap, String id, @NonNull GoBILDA gobildaType) {
        super(hardwareMap, id, gobildaType);
    }

    @Override
    public void set(double output) {
        if (output > 0) {
            output = Math.floor(output * roundingPoint) / roundingPoint;
        } else if (output < 0) {
            output = Math.ceil(output * roundingPoint) / roundingPoint;
        } else {
            output = 0;
        }

        if (currentOutput != output || (currentOutput != 0 && output == 0)) {
            super.set(output);
            currentOutput = output;
        }
    }
}
