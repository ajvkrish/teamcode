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
@Autonomous(name = "Autonomous Encoder Drive Test")
public class AutonomousPractice extends LinearOpMode{
    private Robot robot;

    private int gyroCalibratedCount = 0;
    private double lastHeading = 0;

    private Position initialPosition;
    private Velocity initialVelocity;

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
        //targetPosition=new Position(DistanceUnit.METER,5,0,0,0);

//        robot.unlock();
//        sleep(1000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

        robot.encoderRun(12,.15);
        sleep(5000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

//        robot.moveDownToStart();
//        sleep(2000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

        robot.encoderTurn(180,.15);
        sleep(5000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

    }

    }
