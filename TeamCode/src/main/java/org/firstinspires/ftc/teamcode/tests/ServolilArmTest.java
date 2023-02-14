package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServolilArmTest extends LinearOpMode {
    public void runOpMode (){

        double servoPosition1, servoPosition2;
        Servo lilArm;
        Servo lilArm2;

        lilArm = hardwareMap.get(Servo.class, "lilArm1");
        lilArm2 = hardwareMap.get(Servo.class, "lilArm2");

        telemetry.addLine("waiting for start");
        telemetry.update();
        servoPosition1 = 0.0470000000000047;
        servoPosition2 = 0.8;

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                servoPosition1 += 0.0001;
            }if(gamepad1.left_bumper){
                servoPosition1 -= 0.0001;
            }if(gamepad2.right_bumper){
                servoPosition2 += 0.0001;
            }if(gamepad2.left_bumper){
                servoPosition2 -= 0.0001;
            }
            lilArm.setPosition(servoPosition1);
            lilArm2.setPosition(servoPosition2);
            telemetry.addData("Servo Pos 1", servoPosition1);
            telemetry.addData("Servo Pos 2", servoPosition2);
            telemetry.update();
        }
    }
}
