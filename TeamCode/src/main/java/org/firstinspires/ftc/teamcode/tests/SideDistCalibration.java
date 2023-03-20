package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SideDistCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double distance;
        DistanceSensor sideDist = hardwareMap.get(DistanceSensor.class, "sideDist");

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            distance = Math.floor((sideDist.getDistance(DistanceUnit.INCH) * 10));
            telemetry.addData("side dist", .1 * distance);
            telemetry.addData("cm", sideDist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
