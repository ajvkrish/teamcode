package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name = "Start Crater / Get One Gold")
public class CraterSideAutonomousTensorOneGold extends LinearOpMode{
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

        waitForStart();

        double finalRuntime=robot.runtime.seconds()+5;
        while(path==0 && finalRuntime>robot.runtime.seconds()){
            path=robot.getGoldVertical();
        }
        // sleep(1000);
        //path=robot.getGoldVertical();


        telemetry.addData("initialized",null);
        telemetry.update();
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
            robot.encoderRun(3,.35);
            sleep(250);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            //        robot.moveDownToStart();
//        sleep(1000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

            robot.encoderTurn(-21.8,.35);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(27,.4);
            sleep(1750);
            idle();
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(-8,.35);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();


            robot.encoderTurn(-68.2,.35);
            sleep(2000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

//            robot.encoderRun(34,.3);
//            sleep(3000);
//            robot.finishMovement();
//            idle();
//            telemetry.addData("run complete","");
//            telemetry.update();
//            idle();

            robot.encoderRun(5,.35);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRunTurn(55,-45,.38,.27);
            sleep(4000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(10,.35);//37
            sleep(1000);//3000
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-135,.35);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.deployTeamMarker();
            idle();
            sleep(1500);
            telemetry.addData("Team Marker Deployed","");
            telemetry.update();
            idle();

            robot.undeployTeamMarker();
            idle();
            sleep(500);
            telemetry.addData("Team Marker Deployed","");
            telemetry.update();
            idle();

            robot.encoderRun(-2,.35);
            sleep(250);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-48,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(55,.6);
            sleep(3000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();
        }
        else if(path==3){
            robot.encoderRun(3,.3);
            sleep(250);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            //        robot.moveDownToStart();
//        sleep(1000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

            robot.encoderTurn(28,.35);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(27,.4);
            sleep(1750);
            idle();
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(-6,.35);
            sleep(1000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();


            robot.encoderTurn(-118,.35);
            sleep(2250);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(23,.35);
            sleep(2000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

//            robot.encoderRunTurn(60,-40,.4,.25);
//            sleep(3500);
//            robot.finishMovement();
//            idle();
//            telemetry.addData("run complete","");
//            telemetry.update();
//            idle();
            robot.encoderRunTurn(50,-45,.38,.24);
            sleep(4000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(23,.35);//37
            sleep(1500);//3000
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-45,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(-8,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-70,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.deployTeamMarker();
            idle();
            sleep(1500);
            telemetry.addData("Team Marker Deployed","");
            telemetry.update();
            idle();

            robot.undeployTeamMarker();
            idle();
            sleep(500);
            telemetry.addData("Team Marker Undeployed","");
            telemetry.update();
            idle();

            robot.encoderTurn(-60,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(65,.6);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

        }
        else{
            robot.encoderRun(25,.4);
            sleep(2000);
//        robot.moveDownToStart();
//        sleep(3000);
//        idle();
//        telemetry.addData("done unlocking","");
//        telemetry.update();
//        idle();

            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(-4,.3);
            sleep(500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-90,.35);
            sleep(2000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(15,.4);
            sleep(1250);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRunTurn(55,-45,.38,.25);
            sleep(4000);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(15,.35);//37
            sleep(1500);//3000
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-135,.35);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.deployTeamMarker();
            idle();
            sleep(1500);
            telemetry.addData("Team Marker Deployed","");
            telemetry.update();
            idle();

            robot.undeployTeamMarker();
            idle();
            sleep(500);
            telemetry.addData("Team Marker Undeployed","");
            telemetry.update();
            idle();

            robot.encoderRun(-2,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderTurn(-48,.35);
            sleep(1500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

            robot.encoderRun(60,.5);
            sleep(2500);
            robot.finishMovement();
            idle();
            telemetry.addData("run complete","");
            telemetry.update();
            idle();

        }

        robot.setWRIST_DOWN();
        idle();
        sleep(2000);
//        robot.encoderTurn(-45,.3);
//        sleep(1250);
//        robot.finishMovement();
//        idle();
//        telemetry.addData("run complete","");
//        telemetry.update();
//        idle();


        //Jason can I touch your face?
        //Type answer here: Are you Beyoncé?


    }

}