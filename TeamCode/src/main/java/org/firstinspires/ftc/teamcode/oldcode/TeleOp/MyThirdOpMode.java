package org.firstinspires.ftc.teamcode.oldcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class MyThirdOpMode extends LinearOpMode {

    DcMotor frontleftMotor;
    DcMotor frontrightMotor;
    DcMotor backleftMotor;
    DcMotor backrightMotor;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleftMotor = hardwareMap.dcMotor.get("FrontLeft_Motor");
        frontrightMotor = hardwareMap.dcMotor.get("FrontRight_Motor");
        backleftMotor = hardwareMap.dcMotor.get("BackLeft_Motor");
        backrightMotor = hardwareMap.dcMotor.get("BackRight_Motor");
        waitForStart();
       while (opModeIsActive()) {
        frontleftMotor.setPower(power);
        frontrightMotor.setPower(-power + 1);
        backleftMotor.setPower(-power + 1);
        backrightMotor.setPower(power);

        sleep(2000);

        power = 0.0;

        frontleftMotor.setPower(power);
        frontrightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(power);
        }
    }
}
