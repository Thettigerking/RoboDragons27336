package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor{
        RED,
        GREEN,
        BLUE,
        PURPLE,
        UNKNOWN,
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "RevColorSensor");
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

           /*
        RED:>0.9, <0.4, <0.2
        BLUE:0.8, 1.6, 1.5
        GREEN:0.5, 1.4, 0.8
        PURPLE:0.6, 1.4, 1.7, 1.9
         */

        /*
        if(normRed > 0.9 && normGreen > 0.4 && normBlue > 0.2) {
            return DetectedColor.RED;
        }else if(normRed < 0.8 && normGreen < 1.6 && normBlue > 1.5) {
            return DetectedColor.BLUE;
        }else if(normRed < 0.5 && normGreen > 0.9 && normBlue < 0.7) {
            return DetectedColor.GREEN;
        }else if(normRed > 0.6 && normGreen < 1.7 && normBlue > 1.9){
            return DetectedColor.PURPLE;
        }else{
            return DetectedColor.UNKNOWN;
        }
         */

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
