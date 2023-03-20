package org.firstinspires.ftc.teamcode.oldcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class ImuTest extends LinearOpMode {
    public void runOpMode() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("imu-Y", imu.getAngularVelocity().yRotationRate);
            telemetry.addData("imu-x", imu.getAngularVelocity().xRotationRate);
            telemetry.addData("imu-z", imu.getAngularVelocity().zRotationRate);
            telemetry.update();
        }
    }
}
