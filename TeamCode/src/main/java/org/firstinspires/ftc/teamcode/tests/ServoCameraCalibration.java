package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoCameraCalibration extends LinearOpMode {
    public static double frontPos = .82, backPos = .82;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo back, front;

        back = hardwareMap.get(Servo.class, "back");
        front = hardwareMap.get(Servo.class, "front");

        front.scaleRange(0,.63);

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            back.setPosition(backPos);
            front.setPosition(frontPos);

            telemetry.addData("angle", 180 - (front.getPosition() * 180) - 50.4);
            telemetry.update();
            //.72
        }
    }
}
