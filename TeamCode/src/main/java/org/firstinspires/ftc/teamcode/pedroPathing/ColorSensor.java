package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//@TeleOp
public class ColorSensor {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }

public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"color");
        colorSensor.setGain(8);
}

public DetectedColor getDetectedColor(Telemetry telemetry){
    NormalizedRGBA colors = colorSensor.getNormalizedColors();

    float normRed, normGreen, normBlue;
    normRed = colors.red / colors.alpha;
    normGreen = colors.green / colors.alpha;
    normBlue = colors.blue / colors.alpha;

//telemetry.addData("red", normRed);
//telemetry.addData("green", normGreen);
//telemetry.addData("blue", normBlue);

/*

red, green, blue

Green =<0.05, >0.09, <0.1
Purple =>0.04, <0.09, >0.08

 */
if (normRed < 0.05 && normGreen > 0.09 && normBlue < 0.1){  // norm RGB values for the green artifact
    return DetectedColor.GREEN;
}
else if (normRed > 0.04 && normGreen < 0.09 && normBlue > 0.08) {  // norm RGB values for the green artifact
    return DetectedColor.PURPLE;
}

else {
    return DetectedColor.UNKNOWN;
}
}

}
