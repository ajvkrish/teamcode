package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.view.Gravity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import android.content.Context;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.io.*;
import java.util.ArrayList;
import java.util.List;

import static com.sun.tools.javac.util.Constants.format;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by TheBARYONS on 9/19/2017.
 */

public class Robot2 {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public ElapsedTime runtime = new ElapsedTime();
    public double setRunTime=0;

    public final DcMotor left, right;
    private final DcMotor lifter;
    public final DcMotor rake;
//    private final Servo gripper1, gripper2;
    private final Servo wrist;
    //private final Servo relicArmGripper;
    private final DcMotor flipper;
    //    private final Servo jewelArm;
//    private final Servo relicClaw;
    public final NormalizedColorSensor colorSensor;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Acceleration gravity;
    private Orientation angles;
    private View relativeLayout;

    private double lifterBottomPosition = 0.0;
    private double lifterCurrentPosition = 0.0;

    private final double RADIUS_OF_SPOOL = 1;
    private final double LOCK_DISTANCE = 2600;
    private final double UNLOCK_DISTANCE = 2900;

    private final double GRIPPER1_CLOSED = .4;
    private final double GRIPPER1_OPEN = 0.0;
    private final double GRIPPER1_PARTIAL_OPEN = 0.15;
    private final double GRIPPER2_CLOSED = .4;
    private final double GRIPPER2_OPEN = 0.0;
    private final double GRIPPER2_PARTIAL_OPEN = 0.15;
    private final double RELIC_CLAW_OPEN = .85;
    private final double RELIC_CLAW_CLOSED = .3;
    private final int FLIPPER_ROTATION = 850;
    private final double LIFTER_RANGE = 0.18;
    private final double LIFTER_TOP = 0.49;
    private final double LIFTER_BOTTOM = LIFTER_TOP + 0.18;
    private final double WRIST_OPEN=1;
    private final double WRIST_CLOSED=0;
    private final int INITIAL_RAKE_POSITION=0;
    private final int OUTSIDE_CRATER=1120;
    private final int TOP_CRATER=1680;
    private final int INSIDE_CRATER=2240;
    private final double changeWristPosition=WRIST_CLOSED-WRIST_CLOSED;
    private final int changeRakePosition=TOP_CRATER-OUTSIDE_CRATER;
    private String colorFile="Colors.txt";

    private boolean gripper1Open = true;
    private boolean gripper2Open = true;
    private boolean gripper1Closed = false;
    private boolean gripper2Closed = false;
    private boolean jewelArmOpen = true;
    private boolean relicClawOpen = false;
    private boolean flipperUp = false;
    private int flipperLastPosition = 0;
    private double flipperLastSpeed = 0.0;
    private boolean flipperHold = false;
    private boolean isFlipping = false;

    private double angle=90;
    private double armLength;
    private double flipperGearRatio=1;
    private int flipperTicks;
    private int finalTicks;
    public boolean doneFlip=false;
    public boolean finishingFlip=false;


    public boolean movingLifterUp=false;
    private boolean dpadControls=false;
    private double lbStartTime=runtime.seconds()+.75;
    private double rbStartTime=runtime.seconds()+.75;
    private boolean relicArmExtended=false;
    private int finalRake=0;
    private int initialRake=0;
    public boolean relicArmExtending=false;
    private float[] hsvValues = new float[3];
    private final float values[] = hsvValues;
    private final int relicArmTicks=10000;
    public boolean rotatingClockwise = false;
    public boolean rotating = false;
    public double rotation = 0;
    private boolean isLocked = false;
    private boolean isUnlocked = false;

    public Position currentPosition;
    public Velocity currentVeloity;
    public Position targetPosition;
    public Acceleration currentAcceleration;

    private double targetHeading=getHeadingDegrees();

    private static final String VUFORIA_KEY = "AdqsriL/////AAAAGbZAbJdSMUntkxl5tkkssouJWQxMpZRhlITr2jr3zG+D/KcRc+UfUm5LCP+arU2SdNHIUWX/KjtZ9U8hohgDvx4IviAxEdNF7nK6W5nkteMmFbf9ZX5V3zqecWlVyl5FYUjOl0XXF6BKCuNZwChfdn3L9FxO2N0qbLOPo+hP2EWsPzfXBAMcxr6X/sKfLUuzAFCI/aq5DLVX0G2x1uogBPifiCmpjNv0Vtt/SYf4ba4QM8TG4Y/+2viyoRXeyYK9QU0+LVulr/wAnzHOocRJxnKXipCYqht2M/Adr+Ha+6xVydiAKYGvKdjnrjhaVaWrXUuHfRkYeySXlPYIQZEBqfICeeajwj/S+NzdbJyOy9g2";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final double robotRadius=9;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VuforiaLocalizer vuforia;


    /*
        1120 ticks per wrev
        4" dia wheels
        4 * pi = 12.566  inches/wrev

        1120 t/wrev / 4pi in/wrev =  89.127 t/in

        wheel from center = 9"
        360 rotation = 9 * 2 * pi = 56.549 in/rrev
        18pi t/in * 89.127 in/rrev = t/rrev = 5040 t/rrev

        5040 t/rrev / 360 d/rrev = 14 t/d

        */
    private final double WHEEL_DIAMETER=3.75;
    private final int GEAR_RATIO=2;
    private final double ROBOT_DIAMETER=20.5;
    private final double DEGREES_PER_ROBOT_REV=360;
    private final double TICKS_PER_MOTOR_REV=537.6;
    private final double TICKS_PER_DEGREES=TICKS_PER_MOTOR_REV/(WHEEL_DIAMETER*Math.PI)*(ROBOT_DIAMETER*Math.PI)/DEGREES_PER_ROBOT_REV;

    public int lfadj=50;
    public int rfadj=50;
    public int lradj=50;
    public int rradj=50;
    private boolean lfBusy=false;
    private boolean rfBusy=false;
    private boolean lrBusy=false;
    private boolean rrBusy=false;

    private double xGravAdjustment=0;
    private double yGravAdjustment=0;

    public Robot2(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        rake = hardwareMap.dcMotor.get("rake");
        wrist = hardwareMap.servo.get("wrist");
//        gripper1 = hardwareMap.servo.get("gripper1");
//        gripper2 = hardwareMap.servo.get("gripper2");
        lifter = hardwareMap.dcMotor.get("lifter");
        //relicArmGripper = hardwareMap.servo.get("");
        flipper = hardwareMap.dcMotor.get("flipper");
//        jewelArm = hardwareMap.servo.get("ja");
//        relicClaw = hardwareMap.servo.get("relicClaw");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs");

        rake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        flipper.setDirection(DcMotorSimple.Direction.REVERSE);
//        rake.setDirection(DcMotorSimple.Direction.REVERSE);
        flipperLastPosition = flipper.getCurrentPosition();


        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

        gravity=imu.getGravity();
        xGravAdjustment=gravity.xAccel;
        yGravAdjustment=gravity.yAccel;
//
//        openGripper1();
//        openGripper2();
//        openWrist();
//        initializeJewelArm();
        //openRelicArmGripper();
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, left, right);
    }

    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, left, right);
    }

    /**
     * @return true if the gyro is fully calibrated, false otherwise
     */
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    public boolean isAccelerometerCalibrated(){
        return imu.isAccelerometerCalibrated();
    }

    public void startAccelerationIntegration(Position position, Velocity velocity, int timeIntertval){
        imu.startAccelerationIntegration(position, velocity, timeIntertval);
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentPosition=imu.getPosition();
        currentVeloity=imu.getVelocity();
        currentAcceleration=imu.getAcceleration();
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        if (angles != null) {
            return angles.firstAngle;
        } else {
            return 0.0;
        }
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _left Left motor
     * @param _right Right motor
     */
    public void setMotors(double _left, double _right) {
        final double scale = maxAbs(1.0, _left, _right);
        left.setPower(_left / scale);
        right.setPower(_right / scale);
    }

//    public void openGripper1(){
//        gripper1.setPosition(GRIPPER1_OPEN);
//        gripper1Closed = false;
//        gripper1Open = true;
//    }
//
//    public void closeGripper1(){
//        gripper1.setPosition(GRIPPER1_CLOSED);
//        gripper1Closed = true;
//        gripper1Open = false;
//    }
//
//    public void partialOpenGripper1(){
//        gripper1.setPosition(GRIPPER1_PARTIAL_OPEN);
//        gripper1Closed = false;
//        gripper1Open = false;
//    }
//
//    public void toggleGripper1(){
//        if(gripper1Open)
//            closeGripper1();
//        else if(gripper1Closed)
//            partialOpenGripper1();
//        else
//            openGripper1();
//    }
//
//    public void toggleGripper1Reverse(){
//        if(gripper1Open)
//            partialOpenGripper1();
//        else if(gripper1Closed)
//            openGripper1();
//        else
//            closeGripper1();
//    }
//
//    public void openGripper2(){
//        gripper2.setPosition(GRIPPER2_OPEN);
//        gripper2Closed = false;
//        gripper2Open = true;
//    }
//
//    public void closeGripper2(){
//        gripper2.setPosition(GRIPPER2_CLOSED);
//        gripper2Closed = true;
//        gripper2Open = false;
//    }
//
//    public void partialOpenGripper2(){
//        gripper2.setPosition(GRIPPER2_PARTIAL_OPEN);
//        gripper2Closed = false;
//        gripper2Open = false;
//    }
//
//    public void toggleGripper2(){
//        if(gripper2Open)
//            closeGripper2();
//        else if(gripper2Closed)
//            partialOpenGripper2();
//        else
//            openGripper2();
//    }
//
//    public void toggleGripper2Reverse(){
//        if(gripper2Open)
//            partialOpenGripper2();
//        else if(gripper2Closed)
//            openGripper2();
//        else
//            closeGripper2();
//    }
//
//    public void checkGrippers(boolean rb, boolean lb) {
//        if(!rb)
//            rbStartTime=runtime.seconds()+.75;
//        if(!lb)
//            lbStartTime=runtime.seconds()+.75;
//        if (lbStartTime < runtime.seconds())
//            openGripper2();
//        if (rbStartTime < runtime.seconds())
//            openGripper1();
//    }

    public void moveLifterUp(){
        lifter.setPower(-.5);
    }

    public void moveLifterDown(){
        lifter.setPower(.5);
    }

    public void stopLifter(){
        lifter.setPower(0);
    }

    public void moveFlipper(double speed) {
        // hold position if speed changed to zero from non-zer (release of dpad button)
        speed*=.85;
        if (Math.abs(speed) < 0.05 && Math.abs(flipperLastSpeed) > 0.05) {
            flipperHold = true;
        }

        // if speed is zero and we're still holding
        if (flipperHold && Math.abs(speed) < 0.5){
            holdFlipper(flipperLastPosition);
        } else {
            if (flipper.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            flipperLastPosition = getFlipperPosition();
            //flipperLastSpeed = speed;
            flipper.setPower(speed);
            flipperHold = false;
        }
    }

    public void holdFlipper(int position) {
        if (flipper.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setPower(1.0);
        flipper.setTargetPosition(position);
    }

    public int getFlipperPosition() {
        return flipper.getCurrentPosition();
    }

    public void toggleFlipper() {
        if (flipper.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (flipperUp) {
            flipper.setPower(-0.3);
            flipper.setTargetPosition(flipper.getCurrentPosition() + FLIPPER_ROTATION);  //1120
        }
        else{
            flipper.setPower(0.3);
            flipper.setTargetPosition(flipper.getCurrentPosition() - FLIPPER_ROTATION);
        }
        flipperUp = !flipperUp;
    }

    public boolean getDpadControls(){
        return dpadControls;
    }



    public void setDpadControlsTrue(){
        dpadControls = true;
    }

    public void setDpadControlsFalse(){
        dpadControls = false;
    }

//    public void moveRake(double power){
//        rake.setPower(power/2.0);
//    }


    public void raiseWristAutomatically() {
        if (rake.getCurrentPosition() > OUTSIDE_CRATER && rake.getCurrentPosition() < TOP_CRATER) {
            wrist.setPosition(WRIST_CLOSED+(rake.getCurrentPosition()-OUTSIDE_CRATER)*changeWristPosition/changeRakePosition);
        }
        else if (rake.getCurrentPosition() < INSIDE_CRATER && rake.getCurrentPosition() >= TOP_CRATER) {
            wrist.setPosition(WRIST_CLOSED+(INSIDE_CRATER-rake.getCurrentPosition())*changeWristPosition/changeRakePosition);
        }
        else
            wrist.setPosition(WRIST_CLOSED);
    }

    public void openWrist(){
        wrist.setPosition(WRIST_OPEN);
    }
    public void closeWrist(){
        wrist.setPosition(WRIST_CLOSED);
    }


    /*public void closeRelicArmGripper(){
        relicArmGripper.setPosition(0);
    }

    public void openRelicArmGripper(){
        relicArmGripper.setPosition(.4);
    }

    public void toggleRelicGripper(){
        if(relicArmGripper.getPosition()==0)
            openRelicArmGripper();
        else
            closeRelicArmGripper();
    }*/

    //----------AUTONOMOUS----------

    public void positionDrive(double x, double y) {
        targetPosition = new Position(DistanceUnit.METER, x, y, 0, 0);
        double changeX = targetPosition.x - currentPosition.x;
        double changeY = targetPosition.y - currentPosition.y;
        double distance = Math.sqrt(Math.pow(changeX,2)+Math.pow(changeY,2));
        double targetAngle = Math.atan(changeY / changeX);
        double currentAngle = getHeading();
        double changeAngle = targetAngle - currentAngle;
        if (currentAngle < (-1 * Math.PI))
            currentAngle += 2 * Math.PI;
        if (currentAngle > (Math.PI))
            currentAngle -= 2 * Math.PI;
        if (targetAngle < (-1 * Math.PI))
            targetAngle += 2 * Math.PI;
        if (targetAngle > (Math.PI))
            targetAngle -= 2 * Math.PI;
        if (changeAngle < (-1 * Math.PI))
            changeAngle += 2 * Math.PI;
        if (changeAngle > (Math.PI))
            changeAngle -= 2 * Math.PI;

        double angleDirection = Math.abs(changeAngle) / changeAngle;
        if (angleDirection > 0){
            while (targetAngle > currentAngle) {
                currentAngle=getHeading();
                changeAngle = targetAngle - currentAngle;
                rotation= (changeAngle)/Math.PI;
                setMotors(rotation*-1, rotation);
                telemetry.addData("Change Angle: ",changeAngle);
                telemetry.update();
            }
        }
        if(angleDirection>0){
            while(targetAngle<currentAngle){
                currentAngle=getHeading();
                changeAngle = targetAngle - currentAngle;
                rotation= (changeAngle)/Math.PI;
                setMotors(rotation*-1, rotation);
                telemetry.addData("Change Angle: ",changeAngle);
                telemetry.update();
            }
        }
        while(distance>.05) {
            distance = Math.sqrt(Math.pow(changeX,2)+Math.pow(changeY,2));
            changeX = targetPosition.x - currentPosition.x;
            changeY = targetPosition.y - currentPosition.y;
            double power = Math.min(distance / .1, .5);
            setMotors(power, power);
            telemetry.addData("Distance from Target: ", distance);
            telemetry.update();
        }
    }

    public void encoderRun(double distance, double power){
        int ticks=(int)(distance*(1/(Math.PI*WHEEL_DIAMETER))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        left.setTargetPosition(ticks+left.getCurrentPosition());
        right.setTargetPosition(ticks+right.getCurrentPosition());
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(power);
        right.setPower(power);
    }

    public void toggleLifter() {
        if (isLocked)
            unlock();
        else if (isUnlocked)
            moveDownToStart();
        else
            lock();
    }

    public void lock(){
        int ticks=(int)(LOCK_DISTANCE*(1/(Math.PI*2*RADIUS_OF_SPOOL))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        lifter.setTargetPosition(ticks + lifter.getCurrentPosition());
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.5);
        isLocked = true;
    }
    public void unlock(){
        int ticks=(int)(UNLOCK_DISTANCE*(1/(Math.PI*2*RADIUS_OF_SPOOL))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        lifter.setTargetPosition(ticks + lifter.getCurrentPosition());
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.5);
        isUnlocked = true;
        isLocked = false;

    }

    public void moveDownToStart(){
        int lockTicks=(int)(LOCK_DISTANCE*(1/(Math.PI*2*RADIUS_OF_SPOOL))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        int unlockTicks=(int)(UNLOCK_DISTANCE*(1/(Math.PI*2*RADIUS_OF_SPOOL))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        lifter.setTargetPosition(lifter.getCurrentPosition() - lockTicks - unlockTicks);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.5);
        isUnlocked = false;
    }

    public void stopWinch(){
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn(double angle, double power){
        double distance=angle*Math.PI/180*robotRadius;
        int ticks=(int)(distance*(1/(Math.PI*WHEEL_DIAMETER))*GEAR_RATIO*TICKS_PER_MOTOR_REV);
        left.setTargetPosition(ticks+left.getCurrentPosition());
        right.setTargetPosition(ticks+right.getCurrentPosition());
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(power);
        right.setPower(power);
    }

    public void finishMovement(){
        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


/*
    public void moveBallArmUp(){
        if(ballArm.getPosition()+.05>1.0)
            ballArm.setPosition(1.0);
        else
            ballArm.setPosition(ballArm.getPosition()+.05);
    }

    public void moveBallArmDown(){
        if(ballArm.getPosition()-.05<0.0)
            ballArm.setPosition(1.0);
        else
            ballArm.setPosition(ballArm.getPosition()+.05);
    }
*/

//    public void openJewelArm(){
//        jewelArmOpen=true;
//        jewelArm.setPosition(.48);
//
//    }
//
//    public void closeJewelArm(){
//        jewelArmOpen=false;
//        jewelArm.setPosition(.16);
//    }
//
//    public void initializeJewelArm(){
//        jewelArm.setPosition(.03);
//    }
//
//
//
//    public void toggleJewelArm() {
//        if(jewelArmOpen)
//            closeJewelArm();
//        else
//            openJewelArm();
//    }

    public void resetEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


//    public void run(double distance, double direction, double speed) {
//        lf.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0)))+lf.getCurrentPosition());
//        rf.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0)))+rf.getCurrentPosition());
//        lr.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0)))+lr.getCurrentPosition());
//        rr.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0)))+rr.getCurrentPosition());
//
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        lf.setPower(speed);
//        rf.setPower(speed);
//        lr.setPower(speed);
//        rr.setPower(speed);
//        }

//    public void runTest(double distance, double direction, double speed) {
//        if((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0))>0)
//            lfadj=150;
//        else
//            lfadj=-150;
//        if((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0))>0)
//            rfadj=150;
//        else
//            rfadj=-150;
//        if((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0))>0)
//            lradj=150;
//        else
//            lradj=-150;
//        if((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0))>0)
//            rradj=150;
//        else
//            rradj=-150;
//
//        lf.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0)))+lf.getCurrentPosition()+lfadj);
//        rf.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0)))+rf.getCurrentPosition()+rfadj);
//        lr.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.cos(direction*Math.PI/180 + Math.PI / 4.0)))+lr.getCurrentPosition()+lradj);
//        rr.setTargetPosition(((int)(distance/4/Math.PI*537.6 * Math.sin(direction*Math.PI/180 + Math.PI / 4.0)))+rr.getCurrentPosition()+rradj);
//
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        lf.setPower(speed);
//        rf.setPower(speed);
//        lr.setPower(speed);
//        rr.setPower(speed);
//
//            while(lf.getPower()!=0 || rf.getPower()!=0 || lr.getPower()!=0 || rr.getPower()!=0) {
//                if(lfadj>0)
//                    if(lf.getCurrentPosition()>lf.getTargetPosition()-lfadj)
//                        lf.setPower(0);
//                else
//                    if(lf.getCurrentPosition()<lf.getTargetPosition()-lfadj)
//                        lf.setPower(0);
//                if(rfadj>0)
//                    if(rf.getCurrentPosition()>rf.getTargetPosition()-rfadj)
//                        rf.setPower(0);
//                else
//                    if(rf.getCurrentPosition()<rf.getTargetPosition()-rfadj)
//                        rf.setPower(0);
//                if(lradj>0)
//                    if(lr.getCurrentPosition()>lr.getTargetPosition()-lradj)
//                        lr.setPower(0);
//                else
//                    if(lr.getCurrentPosition()<lr.getTargetPosition()-lradj)
//                        lr.setPower(0);
//                if(rradj>0)
//                    if(rr.getCurrentPosition()>rr.getTargetPosition()-rradj)
//                        rr.setPower(0);
//                else
//                    if(rr.getCurrentPosition()<rr.getTargetPosition()-rradj)
//                        rr.setPower(0);
//            }
     /*   while (lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy()) {
            if(!lf.isBusy())
                lf.setPower(0);
            if(!rf.isBusy())
                rf.setPower(0);
            if(!lr.isBusy())
                lr.setPower(0);
            if(!rr.isBusy())
                rr.setPower(0);

            telemetry.addData("Motors in use",null);
        }
        telemetry.addData("Motors finished",null);
*/

//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

//    public void runComplete(){
//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

//    public void rotate(double degrees, double speed){
//        if(degrees>0) {
//            lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition());
//            rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition());
//            lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition());
//            rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition());
//        }
//        else{
//            lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition());
//            rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition());
//            lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition());
//            rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition());
//        }
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(degrees>0) {
//            lf.setPower(speed);
//            rf.setPower(speed);
//            lr.setPower(speed);
//            rr.setPower(speed);
//        }
//        else{
//            lf.setPower(speed*-1);
//            rf.setPower(speed*-1);
//            lr.setPower(speed*-1);
//            rr.setPower(speed*-1);
//        }
//        runtime.reset();
//    }

//    public void rotateTest2(double degrees, double speed){
//
//            if ((int) (degrees * TICKS_PER_DEGREES) > 0)
//                lfadj = 25;
//            else
//                lfadj = -25;
//            if ((int) (degrees * TICKS_PER_DEGREES * -1) > 0)
//                rfadj = 25;
//            else
//                rfadj = -25;
//            if ((int) (degrees * TICKS_PER_DEGREES) > 0)
//                lradj = 25;
//            else
//                lradj = -25;
//            if ((int) (degrees * TICKS_PER_DEGREES * -1) > 0)
//                rradj = 25;
//            else
//                rradj = -25;
//
//
//        //if(degrees>0) {
//            lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition()+lfadj);
//            rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition()+rfadj);
//            lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition()+lradj);
//            rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition()+rradj);
//        /*}
//        else{
//            lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition());
//            rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition());
//            lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition());
//            rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition());
//        }*/
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(degrees>0) {
//            lf.setPower(speed);
//            rf.setPower(speed);
//            lr.setPower(speed);
//            rr.setPower(speed);
//        }
//        else{
//            lf.setPower(speed*-1);
//            rf.setPower(speed*-1);
//            lr.setPower(speed*-1);
//            rr.setPower(speed*-1);
//        }
//        runtime.reset();
//
//        while(lf.getPower()!=0 || rf.getPower()!=0 || lr.getPower()!=0 || rr.getPower()!=0) {
//            if(lfadj>0) {
//                if (lf.getCurrentPosition() > lf.getTargetPosition() - lfadj)
//                    lf.setPower(0);
//            }else
//                if(lf.getCurrentPosition()<lf.getTargetPosition()-lfadj)
//                    lf.setPower(0);
//            if(rfadj>0) {
//                if (rf.getCurrentPosition() > rf.getTargetPosition() - rfadj)
//                    rf.setPower(0);
//            }else
//                if(rf.getCurrentPosition()<rf.getTargetPosition()-rfadj)
//                    rf.setPower(0);
//            if(lradj>0) {
//                if (lr.getCurrentPosition() > lr.getTargetPosition() - lradj)
//                    lr.setPower(0);
//            }else
//                if(lr.getCurrentPosition()<lr.getTargetPosition()-lradj)
//                    lr.setPower(0);
//            if(rradj>0) {
//                if (rr.getCurrentPosition() > rr.getTargetPosition() - rradj)
//                    rr.setPower(0);
//            }else
//                if(rr.getCurrentPosition()<rr.getTargetPosition()-rradj)
//                    rr.setPower(0);
//        }
//        /*    while (lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy()) {
//            if(!lf.isBusy())
//                lf.setPower(0);
//            if(!rf.isBusy())
//                rf.setPower(0);
//            if(!lr.isBusy())
//                lr.setPower(0);
//            if(!rr.isBusy())
//                rr.setPower(0);
//
//            telemetry.addData("Motors in use",null);
//        }
//        telemetry.addData("Motors finished",null);
//*/
//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//    public void rotateTest(double degrees, double speed){
//
//        if ((int) (degrees * TICKS_PER_DEGREES) > 0)
//            lfadj = 50;
//        else
//            lfadj = -50;
//        if ((int) (degrees * TICKS_PER_DEGREES * -1) > 0)
//            rfadj = 50;
//        else
//            rfadj = -50;
//        if ((int) (degrees * TICKS_PER_DEGREES) > 0)
//            lradj = 50;
//        else
//            lradj = -50;
//        if ((int) (degrees * TICKS_PER_DEGREES * -1) > 0)
//            rradj = 50;
//        else
//            rradj = -50;
//
//
//        //if(degrees>0) {
//        lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition()+lfadj);
//        rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition()+rfadj);
//        lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition()+lradj);
//        rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition()+rradj);
//        /*}
//        else{
//            lf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lf.getCurrentPosition());
//            rf.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rf.getCurrentPosition());
//            lr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES))+lr.getCurrentPosition());
//            rr.setTargetPosition(((int) (degrees * TICKS_PER_DEGREES * -1))+rr.getCurrentPosition());
//        }*/
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(degrees>0) {
//            lf.setPower(speed);
//            rf.setPower(speed);
//            lr.setPower(speed);
//            rr.setPower(speed);
//        }
//        else{
//            lf.setPower(speed*-1);
//            rf.setPower(speed*-1);
//            lr.setPower(speed*-1);
//            rr.setPower(speed*-1);
//        }
//        runtime.reset();
//
//        while(lf.getPower()!=0 || rf.getPower()!=0 || lr.getPower()!=0 || rr.getPower()!=0) {
//            if(lfadj>0) {
//                if (lf.getCurrentPosition() > lf.getTargetPosition())
//                    lf.setPower(0);
//            }else
//            if(lf.getCurrentPosition()<lf.getTargetPosition())
//                lf.setPower(0);
//            if(rfadj>0) {
//                if (rf.getCurrentPosition() > rf.getTargetPosition())
//                    rf.setPower(0);
//            }else
//            if(rf.getCurrentPosition()<rf.getTargetPosition())
//                rf.setPower(0);
//            if(lradj>0) {
//                if (lr.getCurrentPosition() > lr.getTargetPosition())
//                    lr.setPower(0);
//            }else
//            if(lr.getCurrentPosition()<lr.getTargetPosition())
//                lr.setPower(0);
//            if(rradj>0) {
//                if (rr.getCurrentPosition() > rr.getTargetPosition())
//                    rr.setPower(0);
//            }else
//            if(rr.getCurrentPosition()<rr.getTargetPosition())
//                rr.setPower(0);
//        }
//        /*    while (lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy()) {
//            if(!lf.isBusy())
//                lf.setPower(0);
//            if(!rf.isBusy())
//                rf.setPower(0);
//            if(!lr.isBusy())
//                lr.setPower(0);
//            if(!rr.isBusy())
//                rr.setPower(0);
//
//            telemetry.addData("Motors in use",null);
//        }
//        telemetry.addData("Motors finished",null);
//*/
//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

    public List<VuforiaTrackable> vuforiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        targetsRoverRuckus.activate();
        return allTrackables;
    }

    public void vuforiaLoop(List<VuforiaTrackable> allTrackables){
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }



    public boolean isBlueOnLeft(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
//        telemetry.addLine()
//                .addData("H", "%.3f", hsvValues[0])
//                .addData("S", "%.3f", hsvValues[1])
//                .addData("V", "%.3f", hsvValues[2]);
//        telemetry.addLine()
//                .addData("a", "%.3f", colors.alpha)
//                .addData("r", "%.3f", colors.red)
//                .addData("g", "%.3f", colors.green)
//                .addData("b", "%.3f", colors.blue);

        /** We also display a conversion of the colors to an equivalent Android color integer.
         * @see Color */
        int color = colors.toColor();
       /* telemetry.addLine("raw Android color: ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
*/
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        color = colors.toColor();

        telemetry.addLine("normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color))
                .addData("blue - red: ", (int)Color.blue(color)-(int)Color.red(color));
        telemetry.update();
//take two values of "blue-red", multiply the most negative one by -1 and average the two numbers, replace the 52 below with the average

        return (((int)Color.blue(color) + 35) > ((int)Color.red(color)));

//        if(((int)Color.red(color))-22>((int)Color.blue(color)))
//            return false;
//        return true;
    }

    public void turnLightOff(SwitchableLight colorSensor){
        colorSensor.enableLight(false);
    }

    public void partialFlip(double degrees){
        flipper.setTargetPosition((int)(flipper.getCurrentPosition()-degrees*(1120.0/360.0)*(1.0/4.0)));
        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setPower(.15);
        runtime.reset();
    }

    public void doneFlip(){
        flipper.setPower(0);
        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void holdFlipper(){
        flipper.setPower(-0.01);
    }

    public void moveFlipperToPosition(int position) {
        flipper.setTargetPosition(position);
        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setPower(.15);
        runtime.reset();
    }

    public void resetFlipper(){
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isFlip(){
        return isFlipping;
    }

    public void startFlip(){
        //
        flipperTicks=(int)(angle/(360)*TICKS_PER_MOTOR_REV*flipperGearRatio);
        finalTicks=flipperTicks+flipper.getCurrentPosition();
        flipper.setTargetPosition(finalTicks);
        isFlipping=true;
    }

    public void flip(){
        //first half constant .6
        //decrease constant acceleration to .2
        double constantVelocity=.6;
        double finalVelocity=.2;
        double acceleration=(Math.pow(finalVelocity,2)-Math.pow(constantVelocity,2)/(flipperTicks));
        double power;
        if(flipper.getCurrentPosition()+flipperTicks/2<finalTicks)
            power=.6;
        else if(flipper.getCurrentPosition()>finalTicks) {
            power = 0;
            isFlipping=false;
            doneFlip=true;
        }
        else
            power=Math.sqrt(Math.pow(constantVelocity,2)+2*(acceleration)*(flipperTicks/2-(finalTicks-flipper.getCurrentPosition())));

        flipper.setPower(power);
    }

    public void startFlipReturn(){
        flipperTicks=(int)(-1*angle/(360)*TICKS_PER_MOTOR_REV*flipperGearRatio);
        finalTicks=flipperTicks+flipper.getCurrentPosition();
        flipper.setTargetPosition(finalTicks);
        flipper.setPower(.6);
        doneFlip=false;
        finishingFlip=true;
    }

    public void flippingBack(){
        if(flipper.getCurrentPosition()<flipper.getTargetPosition()) {
            flipper.setPower(0);
            finishingFlip=false;
        }
    }

    public void setFlipperPower(double power){
        flipper.setPower(power/8);
    }









    public VuforiaTrackable vuMarkInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdqsriL/////AAAAGbZAbJdSMUntkxl5tkkssouJWQxMpZRhlITr2jr3zG+D/KcRc+UfUm5LCP+arU2SdNHIUWX/KjtZ9U8hohgDvx4IviAxEdNF7nK6W5nkteMmFbf9ZX5V3zqecWlVyl5FYUjOl0XXF6BKCuNZwChfdn3L9FxO2N0qbLOPo+hP2EWsPzfXBAMcxr6X/sKfLUuzAFCI/aq5DLVX0G2x1uogBPifiCmpjNv0Vtt/SYf4ba4QM8TG4Y/+2viyoRXeyYK9QU0+LVulr/wAnzHOocRJxnKXipCYqht2M/Adr+Ha+6xVydiAKYGvKdjnrjhaVaWrXUuHfRkYeySXlPYIQZEBqfICeeajwj/S+NzdbJyOy9g2";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        return relicTemplate;
    }
//
//    public RelicRecoveryVuMark vuMarkId(RelicRecoveryVuMark vuMark, VuforiaTrackable relicTemplate) {
//        runtime.reset();
//        while(vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.seconds()<3){
//            vuMark = RelicRecoveryVuMark.from(relicTemplate);
//        }
//
//        return vuMark;
//    }
//
//    public RelicRecoveryVuMark vuMarkIdWait(RelicRecoveryVuMark vuMark ,double wait) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = "AdqsriL/////AAAAGbZAbJdSMUntkxl5tkkssouJWQxMpZRhlITr2jr3zG+D/KcRc+UfUm5LCP+arU2SdNHIUWX/KjtZ9U8hohgDvx4IviAxEdNF7nK6W5nkteMmFbf9ZX5V3zqecWlVyl5FYUjOl0XXF6BKCuNZwChfdn3L9FxO2N0qbLOPo+hP2EWsPzfXBAMcxr6X/sKfLUuzAFCI/aq5DLVX0G2x1uogBPifiCmpjNv0Vtt/SYf4ba4QM8TG4Y/+2viyoRXeyYK9QU0+LVulr/wAnzHOocRJxnKXipCYqht2M/Adr+Ha+6xVydiAKYGvKdjnrjhaVaWrXUuHfRkYeySXlPYIQZEBqfICeeajwj/S+NzdbJyOy9g2";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//
//        relicTrackables.activate();
//
//        runtime.reset();
//        while(runtime.seconds()<wait){
//            if(vuMark == RelicRecoveryVuMark.UNKNOWN)
//                vuMark = RelicRecoveryVuMark.from(relicTemplate);
//        }
//
//        return vuMark;
//    }
//
//    public RelicRecoveryVuMark vuMarkIdOnce(RelicRecoveryVuMark vuMark) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = "AdqsriL/////AAAAGbZAbJdSMUntkxl5tkkssouJWQxMpZRhlITr2jr3zG+D/KcRc+UfUm5LCP+arU2SdNHIUWX/KjtZ9U8hohgDvx4IviAxEdNF7nK6W5nkteMmFbf9ZX5V3zqecWlVyl5FYUjOl0XXF6BKCuNZwChfdn3L9FxO2N0qbLOPo+hP2EWsPzfXBAMcxr6X/sKfLUuzAFCI/aq5DLVX0G2x1uogBPifiCmpjNv0Vtt/SYf4ba4QM8TG4Y/+2viyoRXeyYK9QU0+LVulr/wAnzHOocRJxnKXipCYqht2M/Adr+Ha+6xVydiAKYGvKdjnrjhaVaWrXUuHfRkYeySXlPYIQZEBqfICeeajwj/S+NzdbJyOy9g2";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//
//        relicTrackables.activate();
//
//        if(vuMark == RelicRecoveryVuMark.UNKNOWN)
//            vuMark = RelicRecoveryVuMark.from(relicTemplate);
//
//
//        return vuMark;
//    }

//    public void fidget(){
//        runtime.reset();
//
//        rf.setTargetPosition(((int)(4*Math.PI*537.6 * Math.cos(Math.PI / 4.0)))+rf.getCurrentPosition());
//        rr.setTargetPosition(((int)(4*Math.PI*537.6 * Math.sin(Math.PI / 4.0)))+rr.getCurrentPosition());
//
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        rf.setPower(.3);
//        rr.setPower(.3);
//
//        while(runtime.seconds()<.5){
//        }
//        rf.setPower(0);
//        rr.setPower(0);
//
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        lf.setTargetPosition(((int)(8*Math.PI*537.6 * Math.cos(Math.PI / 4.0)))+rf.getCurrentPosition());
//        lr.setTargetPosition(((int)(8*Math.PI*537.6 * Math.sin(Math.PI / 4.0)))+rr.getCurrentPosition());
//
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        lf.setPower(.3);
//        lr.setPower(.3);
//
//        while(runtime.seconds()<1.25){
//        }
//        lf.setPower(0);
//        lr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        rf.setTargetPosition(((int)(8*Math.PI*537.6 * Math.cos(Math.PI / 4.0)))+rf.getCurrentPosition());
//        rr.setTargetPosition(((int)(8*Math.PI*537.6 * Math.sin(Math.PI / 4.0)))+rr.getCurrentPosition());
//
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        rf.setPower(.3);
//        rr.setPower(.3);
//
//        while(runtime.seconds()<2){
//        }
//        rf.setPower(0);
//        rr.setPower(0);
//
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        lf.setTargetPosition(((int)(4*Math.PI*537.6 * Math.cos(Math.PI / 4.0)))+rf.getCurrentPosition());
//        lr.setTargetPosition(((int)(4*Math.PI*537.6 * Math.sin(Math.PI / 4.0)))+rr.getCurrentPosition());
//
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        lf.setPower(.3);
//        lr.setPower(.3);
//
//        while(runtime.seconds()<2.5){
//        }
//        lf.setPower(0);
//        lr.setPower(0);
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

//    public void openRelicClaw(){
//        relicClawOpen=true;
//        relicClaw.setPosition(RELIC_CLAW_OPEN);
//    }
//
//    public void closeRelicClaw(){
//        relicClawOpen=false;
//        relicClaw.setPosition(RELIC_CLAW_CLOSED);
//    }
//
//    public void relicClawToggle(){
//        if(relicClawOpen)
//            closeRelicClaw();
//        else
//            openRelicClaw();
//    }
//
//    public void incrementRelicClawUp(){
//        if(relicClaw.getPosition()+.05>1)
//            relicClaw.setPosition(1);
//        else
//            relicClaw.setPosition(relicClaw.getPosition()+.05);
//    }
//
//    public void incrementRelicClawDown(){
//        if(relicClaw.getPosition()-.05<0)
//            relicClaw.setPosition(0);
//        else
//            relicClaw.setPosition(relicClaw.getPosition()-.05);
//    }
//
//    public void relicClawTelemetry(){
//        telemetry.addData("Relic Claw Position: ", relicClaw.getPosition());
//    }


//

//    public void moveRakeFast(double finalPosition){
//        rake.setPower(.8);
//    }
//
//    public void relicArmTelemetry(){
//        telemetry.addData("Relic Arm Ticks: ",rake.getCurrentPosition());
//    }

    /*public void wheelCollection(double power)
    {
        glyphWheel.setPower(power);
    }*/

    public void faceSideCryptoBox() {
        rotating = true;
        if(getHeadingDegrees() > 0 || getHeadingDegrees() < -180) {
            rotation = -.3;
        } else {
            rotation = .3;
            rotatingClockwise = true;
        }
    }

    public void rotatingTowardsCryptoBox() {
        if(getHeadingDegrees() < -180){
            if (rotatingClockwise){
                if (getHeadingDegrees() < -163) {
                    rotation = 0;
                    rotating = false;
                    rotatingClockwise = false;
                }
            }
            else {
                if (getHeadingDegrees() > 163) {
                    rotation = 0;
                    rotating = false;
                }
            }
        }
        else{
            if (rotatingClockwise){
                if (getHeadingDegrees() < -163) {
                    rotation = 0;
                    rotating = false;
                    rotatingClockwise = false;
                }
            }
            else {
                if (getHeadingDegrees() > 163) {
                    rotation = 0;
                    rotating = false;
                }
            }
        }
    }

    public void imuTelemetry(){
        telemetry.addData("Gravity: ", imu.getGravity());
        telemetry.addData("Velocity: ", imu.getVelocity());
        telemetry.addData("Position: ", imu.getPosition());
    }

    public void autoBalance(){
        //LEFT=Y POSITIVE; RIGHT=Y NEGATIVE; FORWARD=X POSITIVE; BACKWARD=X NEGATIVE
        gravity=imu.getGravity();
        gravity.xAccel=(gravity.xAccel-xGravAdjustment)/10.0;
        gravity.yAccel=(gravity.yAccel-yGravAdjustment)/10.0;
    }

    public double autoBalanceRotation(){
        return Math.atan2(gravity.yAccel, -gravity.xAccel);
    }
    public double autoBalanceSpeed(){
        return Math.sqrt(Math.pow(gravity.xAccel,4)+Math.pow(gravity.yAccel,4));
    }

    public void shutDownColorSensor()
    {
        colorSensor.close();
    }


}