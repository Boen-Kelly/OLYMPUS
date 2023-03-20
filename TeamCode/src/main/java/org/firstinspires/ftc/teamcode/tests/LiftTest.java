package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends LinearOpMode {
    public void runOpMode (){
        DcMotor Lift1, Lift2, arm;
        int height = 0;
        int armPos = 0;

        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");

        arm = hardwareMap.get(DcMotor.class, "arm");
        Lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();

        Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()){
            if(gamepad1.right_trigger > 0){
                height += gamepad1.right_trigger * 10;
            }else if(gamepad1.left_trigger > 0){
                height -= gamepad1.left_trigger * 10;
            }

            if(gamepad1.right_bumper){
                armPos += 10;
            }else if (gamepad1.left_bumper){
                armPos -=10;
            }


            Lift1.setTargetPosition(height);
            Lift2.setTargetPosition(Lift1.getTargetPosition());
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           Lift1.setPower(.5);
            Lift2.setPower(Lift1.getPower());

            arm.setTargetPosition(armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);



            telemetry.addData("Lift1Pos", Lift1.getCurrentPosition());
            telemetry.addData("Lift2Pos", Lift2.getCurrentPosition());
            telemetry.addData("height", height);
            telemetry.update();
        }
    }
}
