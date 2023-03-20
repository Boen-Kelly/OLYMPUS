package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class ArmStabilization extends LinearOpMode {
    public void runOpMode(){
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        DcMotor armL, armR;

        armL = hardwareMap.get(DcMotor.class, "arm");
        armR = hardwareMap.get(DcMotor.class, "arm2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        armR.setDirection(DcMotorSimple.Direction.REVERSE);

        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(parameters);

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            telemetry.addLine("not calibrated");
            telemetry.update();
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            if(imu.getAngularOrientation().thirdAngle < (90 - 10)){
                armL.setTargetPosition(-150);
                armR.setTargetPosition(armL.getTargetPosition());
                armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armR.setMode(armL.getMode());
                armL.setPower(1);
                armR.setPower(armL.getPower());
            }else{
                armL.setTargetPosition(0);
                armR.setTargetPosition(armL.getTargetPosition());
                armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armR.setMode(armL.getMode());
                armL.setPower(1);
                armR.setPower(armL.getPower());
            }

            telemetry.addData("first angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("second angle", imu.getAngularOrientation().secondAngle);
            telemetry.addData("third angle", imu.getAngularOrientation().thirdAngle);
            telemetry.update();
        }
    }
}
