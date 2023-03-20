package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class ScissorLift extends LinearOpMode {
    public void runOpMode(){
        DcMotor bottom, top;
        final int type = 0;

        bottom = hardwareMap.get(DcMotor.class, "bottom");
        top = hardwareMap.get(DcMotor.class, "top");

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            bottom.setPower(gamepad1.left_stick_y);
            top.setPower(bottom.getPower());
        }
    }
}
