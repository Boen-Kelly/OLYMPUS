package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
@Disabled
public class EncoderTests extends LinearOpMode {
    public void runOpMode(){
        DcMotor bl,br,fl,fr,intake,duckWheel,leftArm,rightArm;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duckWheel = hardwareMap.get(DcMotor.class, "Rim");
        leftArm = hardwareMap.get(DcMotor.class, "arm");
        rightArm = hardwareMap.get(DcMotor.class, "arm2");

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("bl encoder", bl.getCurrentPosition());
            telemetry.addData("br encoder", br.getCurrentPosition());
            telemetry.addData("fl encoder", fl.getCurrentPosition());
            telemetry.addData("fr encoder", fr.getCurrentPosition());
            telemetry.addData("intake encoder", intake.getCurrentPosition());
            telemetry.addData("duckWheel encoder", duckWheel.getCurrentPosition());
            telemetry.addData("leftArm encoder", leftArm.getCurrentPosition());
            telemetry.addData("rightArm encoder", rightArm.getCurrentPosition());
            telemetry.update();
        }
    }
}
