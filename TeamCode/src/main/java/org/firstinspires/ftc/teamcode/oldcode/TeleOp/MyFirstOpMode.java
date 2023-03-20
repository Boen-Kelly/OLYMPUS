package org.firstinspires.ftc.teamcode.oldcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class MyFirstOpMode extends LinearOpMode {

    DcMotor frontleftMotor;
    DcMotor frontrightMotor;
    DcMotor backleftMotor;
    DcMotor backrightMotor;
    BNO055IMU IMU;

    double power = 0.0;
    double rotation = 0.0;
    double strafe = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleftMotor = hardwareMap.dcMotor.get("fl");
        frontrightMotor = hardwareMap.dcMotor.get("fr");
        backleftMotor = hardwareMap.dcMotor.get("bl");
        backrightMotor = hardwareMap.dcMotor.get("br");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMU =hardwareMap.get(BNO055IMU.class,"imu");
        IMU.initialize(parameters);

        frontleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!IMU.isGyroCalibrated()) {
            telemetry.addData("status","IMU is not calibrated");
            telemetry.update();

        }
        telemetry.addData("status","IMU is calibrated");
        telemetry.update();

        waitForStart();
       while (opModeIsActive()) {
        frontleftMotor.setPower(power+strafe+rotation);
        frontrightMotor.setPower(power-strafe-rotation);
        backleftMotor.setPower(power-strafe+rotation);
        backrightMotor.setPower(power+strafe-rotation);


        power = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        telemetry.addData("X axis rotation",IMU.getAngularOrientation().firstAngle);
        telemetry.addData("Y axis rotation",IMU.getAngularOrientation().secondAngle);
        telemetry.addData("Z axis rotation",IMU.getAngularOrientation().thirdAngle);
        telemetry.update();
       }
    }
}
