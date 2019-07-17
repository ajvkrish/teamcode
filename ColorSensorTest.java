package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Disabled
@Autonomous(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode{
    private ColorSensor cs;

    @Override
    public void runOpMode() {
        cs=hardwareMap.colorSensor.get("cs");
        waitForStart();


    }

//    public boolean isBlueOnLeft(){
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        Color.colorToHSV(colors.toColor(), hsvValues);
////        telemetry.addLine()
////                .addData("H", "%.3f", hsvValues[0])
////                .addData("S", "%.3f", hsvValues[1])
////                .addData("V", "%.3f", hsvValues[2]);
////        telemetry.addLine()
////                .addData("a", "%.3f", colors.alpha)
////                .addData("r", "%.3f", colors.red)
////                .addData("g", "%.3f", colors.green)
////                .addData("b", "%.3f", colors.blue);
//
//        /** We also display a conversion of the colors to an equivalent Android color integer.
//         * @see Color */
//        int color = colors.toColor();
//       /* telemetry.addLine("raw Android color: ")
//                .addData("a", "%02x", Color.alpha(color))
//                .addData("r", "%02x", Color.red(color))
//                .addData("g", "%02x", Color.green(color))
//                .addData("b", "%02x", Color.blue(color));
//*/
//        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
//        colors.red   /= max;
//        colors.green /= max;
//        colors.blue  /= max;
//        color = colors.toColor();
//
//        telemetry.addLine("normalized color:  ")
//                .addData("a", "%02x", Color.alpha(color))
//                .addData("r", "%02x", Color.red(color))
//                .addData("g", "%02x", Color.green(color))
//                .addData("b", "%02x", Color.blue(color))
//                .addData("blue - red: ", (int)Color.blue(color)-(int)Color.red(color));
//        telemetry.update();
////take two values of "blue-red", multiply the most negative one by -1 and average the two numbers, replace the 52 below with the average
//
//        return (((int)Color.blue(color) + 35) > ((int)Color.red(color)));
//
////        if(((int)Color.red(color))-22>((int)Color.blue(color)))
////            return false;
////        return true;
//        }



}
