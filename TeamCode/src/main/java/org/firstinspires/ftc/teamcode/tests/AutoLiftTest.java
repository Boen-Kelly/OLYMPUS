package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.classes.LiftArm;

@TeleOp
//@Disabled
public class AutoLiftTest extends LinearOpMode {
    public void runOpMode(){
        LiftArm lift = new LiftArm(hardwareMap);
        Thread liftThread = new Thread(lift);

        liftThread.start();
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                lift.lift(500, true);
            }else if(gamepad1.b){
                lift.lift(250,false);
            }else if(gamepad1.y){
                lift.lift(0, true);
            }else if(gamepad1.x){
                lift.lift(0, false);
            }

            telemetry.addData("lift", lift);
            telemetry.update();
        }
        liftThread.interrupt();

    }
}
