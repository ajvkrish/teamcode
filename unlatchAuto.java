package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name = "Basic Unlatch Park Crater Side")
public class unlatchAuto extends LinearOpMode{
    private Robot robot;

    private int gyroCalibratedCount = 0;
    private double lastHeading = 0;
    private int path=0; //1=left, 2=middle, 3=right
    private double distanceTraveled;

    @Override
    public void runOpMode() {
        robot= new Robot(hardwareMap, telemetry);
        robot.resetEncoders();
        robot.runUsingEncoders();
        while (!robot.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        //robot.startAccelerationIntegration(initialPosition, initialVelocity,5);
        robot.loop();
        lastHeading = robot.getHeading();
        telemetry.addData("initialized",null);
        telemetry.update();
        waitForStart();

        robot.unlatch();
        sleep(100);
        idle();
        telemetry.addData("done unlatching","");
        telemetry.update();
        idle();

        robot.unlock();
        sleep(3000);
        idle();
        telemetry.addData("done unlocking","");
        telemetry.update();
        idle();


        robot.encoderRun(40,.2);
        sleep(10000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

    }

}
