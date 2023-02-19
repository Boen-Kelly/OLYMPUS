package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AllTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor bl, br, fl, fr, arm, liftBottom, liftTop;
        Servo frontCam, backCam, lilArm, lilArm2;
        CRServo intake;
        DistanceSensor back, front;
        RevBlinkinLedDriver leds;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "fl");
        fl = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        arm = hardwareMap.get(DcMotor.class, "arm");

        liftBottom = hardwareMap.get(DcMotor.class, "Lift1");
        liftTop = hardwareMap.get(DcMotor.class, "Lift2");
        intake = hardwareMap.get(CRServo.class, "Slurper");
        lilArm = hardwareMap.get(Servo.class, "lilArm1");
        lilArm2 = hardwareMap.get(Servo.class, "lilArm2");

        back = hardwareMap.get(DistanceSensor.class, "backDist");
        front = hardwareMap.get(DistanceSensor.class, "frontDist");

        frontCam = hardwareMap.get(Servo.class, "front");
        backCam = hardwareMap.get(Servo.class, "back");

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        
        telemetry.addLine("waiting for start");
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()){

            if(gamepad1.right_bumper){
                lilArm2.setPosition(0.3225000000000471);
            }else{
                lilArm2.setPosition(0.8);
            }
            if(gamepad1.left_bumper){
                lilArm.setPosition(0.473);
            }else{
                lilArm.setPosition(0.04700000000000047);
            }

            if(gamepad1.y){
                intake.setPower(1);
            }else{
                intake.setPower(0);
            }

            if(gamepad1.a){
                frontCam.setPosition(1);
            }else{
                frontCam.setPosition(0);
            }

            if(gamepad1.b){
                backCam.setPosition(1);
            }else{
                backCam.setPosition(0);
            }

            liftBottom.setPower(gamepad1.right_trigger);
            liftTop.setPower(gamepad1.left_trigger);

            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

            if(gamepad1.dpad_up){
                bl.setPower(1);
            }

            if(gamepad1.dpad_down){
                br.setPower(1);
            }

            if(gamepad1.dpad_right){
                fl.setPower(1);
            }

            if(gamepad1.dpad_left){
                fr.setPower(1);
            }

            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("front distance sensor", front.getDistance(DistanceUnit.INCH));
            telemetry.addData("back distance sensor", back.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
