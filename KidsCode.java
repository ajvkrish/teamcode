package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

@TeleOp(name = "Kids Drive", group = "Two controller")
public class KidsCode extends OpMode {

    private Robot robot;
    private Controller controller1;
    private Controller controller2;
    private int gyroCalibratedCount = 0;

    private boolean gyroRotate;
    private boolean tankDrive=false;
    private double targetAngle;
    private double currentAngle;
    private boolean movingCW=true;
    private double angleDifference;
    private double angleDirection;
    //    private Position initialPosition;
//    private Velocity initialVelocity;
    private boolean bLast=false;
    private double governor = 0.75;
    private double rotationGovernor = 0.3;
    private double lastHeading = 0.0;
    private boolean disableDrive=false;

    //    private boolean accelerometerCalibrated=false;

//    private List<VuforiaTrackable> allTrackables=null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        //        initialPosition= new Position(DistanceUnit.METER,0,0,0,0);
//        initialVelocity= new Velocity(DistanceUnit.METER,0,0,0,0);
        // allTrackables=robot.vuforiaInit();
        robot.shutdownTensorFlow();
    }

    @Override
    public void init_loop() {
        controller1.update();
        controller2.update();


        if (robot.isGyroCalibrated()) {
            robot.loop();
            lastHeading = robot.getHeading();
        }

//        if (robot.isAccelerometerCalibrated() && !accelerometerCalibrated) {
//            robot.startAccelerationIntegration(initialPosition, initialVelocity, 5);
//            accelerometerCalibrated=true;
//        }
//        //robot.vuforiaLoop(allTrackables);
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
        // telemetry.addData("Accelerometer Ready?", robot.isAccelerometerCalibrated() ? "YES" : "no.");
        telemetry.update();
    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();
        robot.loop();
        //robot.vuforiaLoop(allTrackables);
        if (controller1.XOnce()) {
            robot.resetHeading();
        }

        double heading = robot.getHeading();
        double headingDelta = 0.0;

        double leftPower=0;
        double rightPower=0;
        if(controller2.dpadRightOnce())
            disableDrive=!disableDrive;
        if(!disableDrive) {
            leftPower = Math.pow(controller1.left_stick_y, 3)/4;
            rightPower = Math.pow(controller1.right_stick_y, 3)/4;

            //PRACTICE 2
            //make one joysick drive controls, pointing right will turn right, left will turn left, forward move forward, back moves back, and use trig functions for scaling
            //values in between, also use pythag for speed to multiply by

//            final double direction = Math.atan2(controller1.left_stick_x, controller1.left_stick_y)/2;
            double direction=(Math.atan2(controller1.left_stick_y,controller1.left_stick_x)+2*Math.PI)%(2*Math.PI);
            final double speed = Math.min(1, Math.sqrt(Math.pow(controller1.left_stick_x,2) + Math.pow(controller1.left_stick_y,2)));
            rightPower=speed*Math.cos(direction-Math.PI/4);
            leftPower=speed *Math.sin(direction-Math.PI/4);

            //PRACTICE CODE
            //if controller2 right stick points in any direction make the robot rotate and stop poinnted in that direction based on the intial heading.
            // The initial heading is considered straight forward on the joystick
            currentAngle=robot.getHeading();
            targetAngle=(Math.atan2(controller1.right_stick_y*-1,controller1.right_stick_x)+3/2*Math.PI)%(2*Math.PI);
            telemetry.addData("Current angle, " ,currentAngle);
            telemetry.addData("Target angle, " ,targetAngle);


            double cwd=currentAngle-targetAngle;
            if(cwd<0)
                cwd+=2*Math.PI;

            double ccwd=targetAngle-currentAngle;
            if(ccwd<0)
                ccwd+=2*Math.PI;
            telemetry.addData("Counterclockwise distance" , ccwd);
            telemetry.addData("Clockwise distance" , cwd);

            double power;
            if(ccwd<cwd)
                movingCW=false;
            else
                movingCW=true;
if(controller1.right_stick_x > 0.05 || controller1.right_stick_x < -0.05 || controller1.right_stick_y > 0.05 || controller1.right_stick_y < -0.05) {

    if (movingCW) {
        leftPower = Math.min(1, Math.pow(cwd, 5))/2;
        rightPower = -1 * Math.min(1, Math.pow(cwd, 5))/2;
    } else {
        leftPower = -1 * Math.min(1, Math.pow(cwd, 5))/2;
        rightPower = Math.min(1, Math.pow(cwd, 5))/2;
    }
}
            telemetry.addData("Left Power: " , leftPower);
            telemetry.addData("Right Power: " , rightPower);

                robot.setMotors(leftPower,rightPower);


        }



        if(controller2.rightBumper()) {
            leftPower = -.2;
            rightPower = .2;
        }
        if(controller2.leftBumper()) {
            leftPower = .2;
            rightPower = -.2;
        }
        //}
        if(robot.flippingOver()){
            leftPower=1;
            rightPower=1;
        }
//        robot.setMotors(leftPower, rightPower);


        if(controller2.XOnce()) {
            if (robot.getTmPosition() == 0)
                robot.deployTeamMarker();
            else
                robot.undeployTeamMarker();
        }

//        if(controller2.rightTriggerPressed())
//            robot.moveLifter(controller2.right_trigger*-1);
//        else if(controller2.leftTriggerPressed())
//            robot.moveLifter(controller2.left_trigger);
//        else
//            robot.moveLifter(0);

        if(controller2.dpadUp())
            robot.setFlipperPower(.5);
        else if(controller2.dpadDown())
            robot.setFlipperPower(-.5);
        else
            robot.setFlipperPower(0);


        if(controller2.XOnce())
            robot.setWRIST_MIDDLE();


        //STATE MACHINE
        if(robot.STATE==0 || robot.STATE==2)
            robot.customExtend(controller2.left_stick_y*-1/2);
        if(robot.STATE==1)
            robot.extend();
        if(robot.STATE==2)
            robot.collect(controller2.B());
        if(robot.STATE==3)
            robot.retract();
        if(robot.STATE==4)
            robot.dump();

        if(controller2.YOnce()){
            robot.startExtend(robot.LOAD_RAKE_TICKS);
        }
        if(controller2.BOnce()){
            robot.startCollection();
        }
        if(controller2.AOnce()){
            robot.startRetraction();
        }
//        if(controller1.rightTriggerPressed()){
//            robot.startExtend(1500);
//        }

        //End of STATE MACHINE Code

        if(controller2.dpadUpOnce())
            robot.incrementWrist();
        if(controller2.dpadDownOnce())
            robot.decrementWrist();

//        telemetry.addData("Rake Current Position: ",robot.getRakePosition());
//        telemetry.addData("Wrist Current Position: ",robot.getWristPosition());
//        telemetry.addData("Flipper Ticks: ",robot.getFlipperPosition());
       // robot.getGyroGravity();
        telemetry.update();
    }
}

