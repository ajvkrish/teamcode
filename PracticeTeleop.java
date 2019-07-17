package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "PracticeTeleop", group = "Two controller")
public class PracticeTeleop extends OpMode {
//basic tank drive
    //initialize servo to down
    //toggle between positions if x once





    private Controller controller1;
    private Controller controller2;
    private PracticeRobot robot;
    @Override
    public void init(){

        robot=new PracticeRobot(hardwareMap, telemetry);
        controller1=new Controller(gamepad1);
        controller2=new Controller(gamepad2);
    }
    @Override
    public void init_loop(){
        robot.iterations++;
        telemetry.addData("Iterations:", robot.iterations);
        telemetry.addData("Current runtime", robot.runtime.seconds());
        telemetry.addData("Iterations per second:", robot.iterations/(robot.runtime.seconds()-robot.initialRunTime));

    }

    @Override
    public void loop(){
        controller1.update();
        controller2.update();

        robot.iterations++;
        telemetry.addData("Iterations:", robot.iterations);
        telemetry.addData("Current runtime", robot.runtime.seconds());
        telemetry.addData("Iterations per second:", robot.iterations/(robot.runtime.seconds()-robot.initialRunTime));

        robot.dcRight.setPower(Math.pow(controller1.right_stick_y,3)/4);
            robot.dcLeft.setPower(Math.pow(controller1.left_stick_y,3)/4);

            if(controller1.XOnce()) {
                robot.toggle();
            }

            if(controller1.A())
                robot.serVelocity(true);

        if(controller1.B())
            robot.serVelocity(false);





    }
}
