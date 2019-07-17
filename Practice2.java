package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//create code in this opmode ✔
//a servo moves approximately 180 degrees(assume setposition(0) makes it go to zero and setposition(1) makes it go to 180 degrees
//like we did on thursday, make a motor that will go between two positions. when u press a, the motor goes to 180 degrees,
//when u press b, it goes to 0. Move both the motor and the servo simultaneously. move the motor very slow and move the servo based
// on its current position, not the end position. Dont just say if(controller1.a) servoo.setposition(1) rather make a function
//that will input the correct servo position for the current motor angle. You can look back at the code we did yesterday but it
//is a bit different. You can create extra methods in this opmode to organize your code if you wish,
//please don't put code in robot class because if u mess something up im not there to fix it. Good luck
//while ur moving to one position dont let them start moving to the other, create a boolean to check if ur currently moving
//and only start moving if its false
//to find the code we did thrusday, go to robot class at the top and do ctrl f, flip()
//useful methods
//motor.getcurrentposition();
//motor.settargetposition((int)ticks);
//motor.setMode(runtoposition);
//motor.setpower(.05);
//servo.setposition(x);


@TeleOp(name="OpMode Practice", group="Iterative OpMode")
@Disabled
public class Practice2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor leftDrivee = null;
    private DcMotor rightDrivee = null;
    private DcMotor slowMotor = null; //new
    private Servo slowServo = null; //new
    private Servo servo = null;
    private Servo servoo = null;
    private Controller controller1;
    private double powerCoefficient = 0;
    private int motorTicks;
    private double motorGearRatio = 5;
    private double motorAngle = 180;
    private final double TICKS_PER_MOTOR_REV=537.6;
    private int finalTicks;
    private boolean isMoving=false;
    private final double teamSlackers = 999;
    private final int maxTicks=2500;
    private final int minTicks=-2500;
    private boolean movingRake=false;
    private DcMotor rakeMotor = null;

    public int STATE=0;
    private double driveTime=0;



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrivee = hardwareMap.get(DcMotor.class, "rightDrivee");
        leftDrivee = hardwareMap.get(DcMotor.class, "left_drivee");
        servo = hardwareMap.get(Servo.class, "servo");
        servoo = hardwareMap.get(Servo.class,"servoo");
        slowServo = hardwareMap.get(Servo.class, "slowServo");
        slowMotor = hardwareMap.get(DcMotor.class, "slowMotor");
        rakeMotor = hardwareMap.get(DcMotor.class, "rakeMotor");

        leftDrivee.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;


        if (controller1.left_trigger > 0) {
            leftDrive.setPower(-1 * controller1.left_trigger);
        } else if (controller1.right_trigger > 0) {
            leftDrive.setPower(1 * controller1.right_trigger);

        } else {
            leftDrive.setPower(0);

        }

        double pos1 = .1;
        double pos2 = .5;
        double pos3 = .8;

        if (servo.getPosition() == pos1)
            servo.setPosition(pos2);
        else if (servo.getPosition() == pos2)
            servo.setPosition(pos3);
        else if (servo.getPosition() == pos3)
            servo.setPosition(pos1);


        powerCoefficient = controller1.right_stick_x;
        leftDrivee.setPower(powerCoefficient);
        rightDrivee.setPower(-1 * powerCoefficient);

        if (controller1.AOnce() == true) {
            servoo.setPosition(servoo.getPosition() + 0.05);

        }
        if (controller1.BOnce() == true) {
            servoo.setPosition(servoo.getPosition() - 0.05);
        }

        if(controller1.AOnce()){
            startSlowMotor();
        }
        if(isMoving) {
            slowMotorFlip();
            moveServo();
        }
        if(STATE==1)
            forwardMoving();
        if(STATE==2)
            backMoving();
        if(STATE==3)
            turnMoving();

        if(controller1.AOnce())
        {
            forward();
        }
        if(controller1.BOnce())
        {
            back();
        }
        if(controller1.YOnce())
            turn();

        if(movingRake)
            moveRake();
        if(movingRake==true)
            moveRakeBack();
        if (controller1.XOnce() && movingRake==false) {
            movingRake = true;
        }

        else if (controller1.XOnce() && movingRake==true) {
            movingRake = false;
        }

        if(movingRake==false); {

            rakeMotor.setPower(0);


        }







    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//create code in this opmode ✔
//a servo moves approximately 180 degrees(assume setposition(0) makes it go to zero and setposition(1) makes it go to 180 degrees
//like we did on thursday, make a motor that will go between two positions. when u press a, the motor goes to 180 degrees,
//when u press b, it goes to 0. Move both the motor and the servo simultaneously. move the motor very slow and move the servo based
// on its current position, not the end position. Dont just say if(controller1.a) servoo.setposition(1) rather make a function
//that will input the correct servo position for the current motor angle. You can look back at the code we did yesterday but it
//is a bit different. You can create extra methods in this opmode to organize your code if you wish,
//please don't put code in robot class because if u mess something up im not there to fix it. Good luck
//while ur moving to one position dont let them start moving to the other, create a boolean to check if ur currently moving
//and only start moving if its false
//to find the code we did thrusday, go to robot class at the top and do ctrl f, flip()
//useful methods
//motor.getcurrentposition();
//motor.settargetposition((int)ticks);
//motor.setMode(runtoposition);
//motor.setpower(.05);
//servo.setposition(x);

        public void moveServo(){

                servoo.setPosition(Math.min((finalTicks-slowMotor.getCurrentPosition())/(double)motorTicks,1));
                }


//

        public void startSlowMotor(){
            motorTicks=(int)(motorAngle/(360)*TICKS_PER_MOTOR_REV*motorGearRatio);
            finalTicks=motorTicks + slowMotor.getCurrentPosition();
            slowMotor.setTargetPosition(finalTicks);
            isMoving=true;
        }
        public void slowMotorFlip(){
            if(slowMotor.getCurrentPosition()<finalTicks)
                slowMotor.setPower(0.05);
            else{
                slowMotor.setPower(0);
                isMoving=false;
            }

        }

        //rake code advanced
        //the rake will continuously move as long as movingRake is true;
        //it moves out to the maxTicks * y position of controller1.left_joystick;
        //toggle movingRake on and off with controller1.X();

        public void moveRake() {
            rakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rakeMotor.setTargetPosition((int) (maxTicks * controller1.left_stick_y));
            rakeMotor.setPower(1);
        }
        public void moveRakeBack() {
            rakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rakeMotor.setTargetPosition((int)(maxTicks*-1*controller1.left_stick_y));
            rakeMotor.setPower(1);
            }

            //STATE MACHINE
            //STATE 0-drive around,
            //press a to go forward 1000 ticks
            //press b to go back for 3 seconds
            //press y to turn for 1000 ticks then go forward 1000 ticks





            public void forward() {
                STATE = 1;
                rightDrivee.setPower(.2);
                leftDrivee.setPower(.2);
                rightDrivee.setTargetPosition(rightDrivee.getCurrentPosition()+1000);
                leftDrivee.setTargetPosition(leftDrivee.getCurrentPosition()+1000);
                }

                public void forwardMoving() {

                if(rightDrivee.getCurrentPosition()>rightDrivee.getTargetPosition()-25)
                    if(leftDrivee.getCurrentPosition()>leftDrivee.getTargetPosition()-25)
                        STATE=0;

                }


                public void back() {

                    STATE = 2;
                    driveTime=runtime.seconds()+3;
                    rightDrivee.setTargetPosition(minTicks);
                    leftDrivee.setTargetPosition(minTicks);
                    rightDrivee.setPower(.2);
                    leftDrivee.setPower(.2);

                }
                public void backMoving() {
                    if(driveTime<runtime.seconds())
                        STATE=0;


                }

                public void turn(){
                    STATE=3;
                    rightDrivee.setPower(.2);
                    leftDrivee.setPower(.2);
                    rightDrivee.setTargetPosition(rightDrivee.getCurrentPosition()-1000);
                    leftDrivee.setTargetPosition(leftDrivee.getCurrentPosition()+1000);


                }

                public void turnMoving() {

                    if(rightDrivee.getCurrentPosition()<rightDrivee.getTargetPosition()+25)
                        if(leftDrivee.getCurrentPosition()>leftDrivee.getTargetPosition()-25)
                            forward();
                }






































}
