package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

public class PracticeAutonomous extends LinearOpMode{
    //make a practice autonomus that moves in a square box 6inches by 6inches, put required methods in practice robot

    private PracticeRobot robot;


    public void runOpMode() {
        robot=new PracticeRobot(hardwareMap,telemetry);


        waitForStart();
       for(int i = 0; i < 4; i++){
           Turn90degrees();
           robot.toggle();
       }


    }

    public void Turn90degrees() {
        robot.encoderRun(6,0.01);
        sleep(1000);
       // robot.finishMovement();
        idle();
        robot.encoderTurn(90, 0.01);
        sleep(1000);
        //robot.finishMovement();
        idle();


    }
}
