package org.firstinspires.ftc.teamcode.robot.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensor.ColorRangefinder;

@TeleOp
@Config
public class ConfigureColorRangefinder extends LinearOpMode {
    public static boolean isDigitalMode = true;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> blue
        only pin1 --> red
        neither   --> no object
         */

        // 20mm or closer requirement
        // yellow
        // red
        // red
        if (isDigitalMode) {
            crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 180 / 360.0 * 255, 270 / 360.0 * 255); // blue
            crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 30 / 360.0 * 255, 90 / 360.0 * 255); // yellow
            crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 35); // 20mm or closer requirement

            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0 / 360.0 * 255, 29 / 360.0 * 255); // red
            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 325 / 360.0 * 255, 360 / 360.0 * 255); // red
            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 30 / 360.0 * 255, 90 / 360.0 * 255); // yellow
            crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 35); // 20mm or closer requirement

        } else {
            crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        }


        waitForStart();

        stop();
    }
}
