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
@Autonomous(name = "Depot Color Sensor")
public class DepotSideAutonomousMain extends LinearOpMode{
    private Robot robot;

    private int gyroCalibratedCount = 0;
    private double lastHeading = 0;
    private int path;
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

//        robot.unlock();
//        sleep(1000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

        robot.encoderTurn(30,.15);
        sleep(2000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        robot.encoderRun(41,.15);
        sleep(1000);
        idle();
//        robot.moveDownToStart();
//        sleep(4000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        robot.encoderTurn(-120,.15);
        sleep(2000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();
        int initialEncoderTicks=robot.getEncoderTicks();

//        while(!robot.isGold()){
//            robot.setMotors(.05,.05);
//            idle();
//        }
//        robot.setMotors(0,0);

        robot.encoderRun(12,.2);
        sleep(2000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        distanceTraveled=robot.calculateDistanceTraveled(initialEncoderTicks);
        double angleTurn = Math.toDegrees(Math.atan(37.714/(14.5-distanceTraveled)));
        double originalAngleTurn=angleTurn;
        if(angleTurn<0)
            angleTurn+=180;

        robot.encoderTurn(angleTurn,.15);
        sleep(5000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        if(angleTurn>90)
            angleTurn=180-angleTurn;
        double distanceToMove=37.714/Math.sin(Math.toRadians(angleTurn));

        robot.encoderRun(distanceToMove,.15);
        sleep(5000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        robot.encoderTurn(-1*(originalAngleTurn+45),.15);
        sleep(5000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

        robot.encoderRun(82,.2);
        sleep(10000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

    }

}
