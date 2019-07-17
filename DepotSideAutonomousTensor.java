package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name = "Start Depot End Other Crater")
public class DepotSideAutonomousTensor extends LinearOpMode{
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
        double finalRuntime=robot.runtime.seconds()+5;
        while(path==0 && finalRuntime>robot.runtime.seconds()){
            path=robot.getGoldVertical();
        }
//        sleep(1000);
//        path=robot.getGoldVertical();

        robot.shutdownTensorFlow();

        robot.unlatch();
        sleep(200);
        idle();
        telemetry.addData("done unlatching","");
        telemetry.update();
        idle();

        robot.unlock();
        sleep(2500);
        idle();
        telemetry.addData("done unlocking","");
        telemetry.update();
        idle();


        if(path==1){
            robot.encoderRun(3,.2);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-35,.2);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(46,.25);
            sleep(6000);
            idle();
//        robot.moveDownToStart();
//            sleep(3000);
//            idle();
//            telemetry.addData("done unlocking","");
//            telemetry.update();
//            idle();
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(70,.2);
            sleep(3000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(26.7,.2);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(190,.25);
            sleep(4000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

        }
        else if(path==3){
            robot.encoderRun(3,.2);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(21.8,.2);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(46,.25);
            sleep(3000);
            idle();
            robot.moveDownToStart();
            sleep(1500);
            idle();
            telemetry.addData("done unlocking","");
            telemetry.update();
            idle();
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-66.8,.2);
            sleep(3000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(26.7,.2);
            sleep(3000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-95,.2);
            sleep(3000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

        }
        else{
            robot.encoderRun(64,.25);
            sleep(2000);
            robot.moveDownToStart();
            sleep(4000);
            idle();
            telemetry.addData("done unlocking","");
            telemetry.update();
            idle();

            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

//            robot.setFlipperPower(.3);
//            sleep(200);
//            idle();
//            robot.setFlipperPower(0);

            robot.encoderTurn(-135,.2);
            sleep(5000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();
        }

        robot.deployTeamMarker();
        idle();
        sleep(500);
        telemetry.addData("Team Marker Deployed","");
        telemetry.update();
        idle();

        robot.undeployTeamMarker();
        idle();
        sleep(500);
        telemetry.addData("Team Marker Undeployed","");
        telemetry.update();
        idle();

        robot.encoderRun(75,.35);
        sleep(10000);
        robot.finishMovement();
        idle();
        telemetry.addData("run complete","");
        telemetry.update();
        idle();

    }

}
