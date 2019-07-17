package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TwoControllerTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Disabled
@Autonomous(name = "TTest Auto")
public class AutonomousTest extends LinearOpMode {
    private Robot robot;
    private TwoControllerTeleOp teleop;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        teleop=new TwoControllerTeleOp();
        telemetry.addData("Initialized","");
        telemetry.update();
        waitForStart();
        teleop.init();
        telemetry.addData("second init","");
        telemetry.update();
        teleop.loop();
        telemetry.addData("Finished","");
        telemetry.update();
    }
}