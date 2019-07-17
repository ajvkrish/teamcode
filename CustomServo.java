package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;


//create a custom servo class that has an added function of being able to set the velocity of a servo not just position
//use ur brains and figure it out, if u need to look at current servo methods just check out the servo class above
public class CustomServo{
    private Servo servo = null;
    private int finalTicks;
    public CustomServo(Servo servo){
        this.servo = servo;
    }
    public void setVelocity(double targetPosition, double velocity){
        double servoDelta;
        // clip the position values so that they never exceed 0..1


        double servoDeltaPerIteration =  1/Robot.ITERATIONSPERSECOND*velocity;
        //positive
        if(servo.getPosition() - servoDeltaPerIteration > targetPosition) {
                servoDelta = servo.getPosition() - servoDeltaPerIteration;

        } else if(servo.getPosition()+servoDeltaPerIteration<targetPosition){
            servoDelta=servo.getPosition()+servoDeltaPerIteration;
        }
        else{
            servoDelta=targetPosition;
        }
        servoDelta = Range.clip(servoDelta, 0, 1);
        servo.setPosition(servoDelta);
    }
    public void getVelocity() throws InterruptedException{
        int time = 300;
        double position1 = servo.getPosition();
        Thread.sleep(time);
        double position2 = servo.getPosition();
        double velocity = (position1-position2)/time;


    }
}
