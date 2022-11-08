package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class IntakeTest extends LinearOpMode {
    public void runOpMode(){
        //You can create the motor and define it in the same line
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            intake.setPower(gamepad1.right_trigger);

            telemetry.addData("motor power", intake.getPower());
            telemetry.update();
        }
    }
}
