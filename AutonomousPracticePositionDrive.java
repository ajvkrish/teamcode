package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Disabled
@Autonomous(name = "Autonomous Position Drive Test")
public class AutonomousPracticePositionDrive extends LinearOpMode{
    private Robot robot;

    private int gyroCalibratedCount = 0;
    private double lastHeading = 0;

    private Position initialPosition;
    private Velocity initialVelocity;

    @Override
    public void runOpMode() {
        telemetry.addData("Begin","");
        telemetry.update();
        robot= new Robot(hardwareMap, telemetry);
        robot.resetEncoders();
        telemetry.addData("Done Resetting Encoders","");
        telemetry.update();
        robot.runUsingEncoders();
        while (!robot.isGyroCalibrated() || !robot.isAccelerometerCalibrated()) {
            sleep(50);
            idle();
            telemetry.addData("Accelerometer Calibrated?: ",robot.isAccelerometerCalibrated());
            telemetry.addData("Gyro Calibrated?: ",robot.isGyroCalibrated());
            telemetry.update();
        }
        telemetry.addData("Done Calibrating","");
        telemetry.update();
        robot.startAccelerationIntegration(initialPosition, initialVelocity,5);
        robot.loop();
        lastHeading = robot.getHeading();
        telemetry.addData("initialized",null);
        telemetry.update();
        waitForStart();
        //targetPosition=new Position(DistanceUnit.METER,5,0,0,0);

        robot.positionDrive(.1,.5);
        telemetry.addData("start run","");
        telemetry.update();
        sleep(1500);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();


    }




}