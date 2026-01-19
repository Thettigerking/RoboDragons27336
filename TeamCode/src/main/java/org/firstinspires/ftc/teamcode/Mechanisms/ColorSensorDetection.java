package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorDetection {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor{
        RED,
        GREEN,
        BLUE,
        PURPLE,
        UNKNOWN,
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        colorSensor.setGain(50);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        if(normGreen > normRed && normGreen > normBlue && normGreen > 1.3
        ) {
            return DetectedColor.GREEN;
        }else if(normRed < normGreen && normBlue > normGreen){
            return DetectedColor.PURPLE;
        }else{
            return DetectedColor.UNKNOWN;
        }

    }

}

//something