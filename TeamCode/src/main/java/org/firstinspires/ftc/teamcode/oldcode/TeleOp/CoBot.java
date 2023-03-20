package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.classes.TapeMeasure;

@Config
@TeleOp(name = "Co-op Tele Opmode")
@Disabled
public class CoBot extends LinearOpMode {
    public static double gateClosed = .1, gateOpen = .37, bottomClosed = 1, bottomOpen = 0, quasiGateOpen = .2, straight, strafe, rotation;
    public static int BOTTOM = 175, MIDDLE = 250, HIGH = 350, targetPos = 0;
    public int SHARED = 175;

    public void runOpMode() {

        DcMotor bl, br, fl, fr, intake, arm, armSupport;
        DcMotorEx duckWheel;
        Servo gate, bottom;
        DigitalChannel limit;
        BNO055IMU imu;
        double slowDown = 1, armPower = 1;
        int bottomLimit = 0, level = 0, levelHeight = 350;
        boolean toggle = false, armState = false, toggle2 = false, toggle3 = false;
        boolean rumble1 = true, rumble2 = true, rumble3 = true;
        String currentTarget = "top";

        ElapsedTime timer = new ElapsedTime();
        Gamepad.RumbleEffect fastBlip;

        fastBlip = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 150)
                .addStep(0, 0, 50)
                .addStep(1, 1, 150)
                .addStep(0, 0, 50)
                .addStep(1, 1, 150)
                .addStep(0, 0, 50)
                .addStep(1, 1, 150)
                .addStep(0, 0, 50)
                .addStep(1, 1, 150)
                .build();

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duckWheel = hardwareMap.get(DcMotorEx.class, "Rim");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armSupport = hardwareMap.get(DcMotor.class, "arm2");
        gate = hardwareMap.get(Servo.class, "gate");
        bottom = hardwareMap.get(Servo.class, "bottom");
        limit = hardwareMap.get(DigitalChannel.class, "limit");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        armSupport.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSupport.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSupport.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limit.setMode(DigitalChannel.Mode.INPUT);

        gate.scaleRange(0, 1);
        bottom.scaleRange(0, 1);

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addLine("not initialized");
            telemetry.update();
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            slowDown = .6;

            straight = gamepad1.left_stick_y * slowDown;
            strafe = (gamepad1.left_stick_x * slowDown);
            rotation = gamepad1.right_stick_x * slowDown * .5;

            bl.setPower(straight + strafe + rotation);
            br.setPower(straight - strafe - rotation);
            fl.setPower(straight - strafe + rotation);
            fr.setPower(straight + strafe - rotation);

            if (gamepad2.right_bumper) {
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (arm.getCurrentPosition() <= -400 && !(gamepad2.left_trigger > 0) && !(imu.getAngularOrientation().thirdAngle < (90 - 15))) {
                arm.setTargetPosition(bottomLimit - 450);
                armSupport.setTargetPosition(arm.getTargetPosition());
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSupport.setMode(arm.getMode());
                arm.setPower(armPower);
                armSupport.setPower(arm.getPower());
            } else if (gamepad2.left_trigger > 0 && limit.getState()) {
                armPower = .5;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(gamepad2.left_trigger * armPower);
                armSupport.setPower(arm.getPower());
                targetPos = arm.getCurrentPosition();
            } else if (gamepad2.right_trigger > 0 && arm.getCurrentPosition() > bottomLimit - 400) {
                armPower = .5;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad2.right_trigger * armPower);
                armSupport.setPower(arm.getPower());
                targetPos = arm.getCurrentPosition();
            } else {
                arm.setTargetPosition(targetPos);
                armSupport.setTargetPosition(arm.getTargetPosition());
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSupport.setMode(arm.getMode());
                arm.setPower(.5);
                armSupport.setPower(arm.getPower());
            }

            if (gamepad2.a) {
                if (toggle) {
                    armState = !armState;
                    toggle = false;
                }
            } else {
                toggle = true;
            }

            if (gamepad2.a && armState) {
                targetPos = bottomLimit - levelHeight;
            } else if (gamepad2.a && !armState) {
                targetPos = bottomLimit;
            }

            if (imu.getAngularOrientation().thirdAngle < (90 - 15)) {
                targetPos = bottomLimit - 100;
                armState = true;
            }

            if ((arm.getCurrentPosition() <= bottomLimit - 300) && gamepad2.y) {
                gate.setPosition(gateOpen);

                if (gamepad2.b) {
                    bottom.setPosition(bottomOpen);
                } else {
                    bottom.setPosition(bottomClosed);
                }
            } else if (arm.getCurrentPosition() >= bottomLimit - 100 && (gamepad2.right_bumper || gamepad2.y)) {
                gate.setPosition(gateOpen);

                if (gamepad2.b) {
                    bottom.setPosition(bottomOpen);
                } else {
                    bottom.setPosition(bottomClosed);
                }
            } else if (bottomLimit - 100 > arm.getCurrentPosition() && arm.getCurrentPosition() > bottomLimit - 300 && gamepad2.y) {
                bottom.setPosition(bottomOpen);

                if (gamepad2.x) {
                    gate.setPosition(gateOpen);
                } else {
                    gate.setPosition(quasiGateOpen);
                }
            } else {
                if (gamepad2.b) {
                    bottom.setPosition(bottomOpen);
                } else {
                    bottom.setPosition(bottomClosed);
                }

                if (gamepad2.x) {
                    gate.setPosition(gateOpen);
                } else {
                    gate.setPosition(gateClosed);
                }
            }

//                if (gamepad2.dpad_left) {
//                    //TODO: I think we should investigate this further in the future, but as for now,
//                    // competition is next week.
////                int startPos = duckWheel.getCurrentPosition();
////                duckWheel.setVelocity(180, AngleUnit.DEGREES);
////                if(duckWheel.getCurrentPosition() + startPos > 400){
////                    duckWheel.setVelocity(360, AngleUnit.DEGREES);
////                }
//                    duckWheel.setVelocity(180, AngleUnit.DEGREES);
//                } else if (gamepad2.dpad_right) {
//                    duckWheel.setVelocity(-180, AngleUnit.DEGREES);
//                } else {
//                    duckWheel.setPower(0);
//                }

            if (!limit.getState()) {
                bottomLimit = arm.getCurrentPosition();
            }

            if (gamepad2.dpad_up) {
                if (toggle2) {
                    level++;
                    toggle2 = false;
                }
            } else if (gamepad2.dpad_down) {
                if (toggle2) {
                    level--;
                    toggle2 = false;
                }
            } else {
                toggle2 = true;
            }

            if (level > 3) {
                level = 0;
            } else if (level < 0) {
                level = 3;
            }

            if (level == 0) {
                levelHeight = SHARED;
                currentTarget = "SHARED SHIPPING HUB";
            } else if (level == 1) {
                levelHeight = BOTTOM;
                currentTarget = "BOTTOM LEVEL";
            } else if (level == 2) {
                levelHeight = MIDDLE;
                currentTarget = "MIDDLE LEVEL";
            } else if (level == 3) {
                levelHeight = HIGH;
                currentTarget = "TOP LEVEL";
            }

//                if(gamepad2.right_bumper){
//                    if(toggle3){
//                        SHARED += 10;
//                        toggle3 = false;
//                    }
//                }else if(gamepad2.left_bumper){
//                    if(toggle3){
//                        SHARED -= 10;
//                        toggle3 = false;
//                    }
//                }else if(gamepad2.right_trigger > 0){
//                    if(toggle3){
//                        SHARED += 20;
//                        toggle3 = false;
//                    }
//                }else if(gamepad2.left_trigger > 0){
//                    if(toggle3){
//                        SHARED -= 20;
//                        toggle3 = false;
//                    }
//                }else{
//                    toggle3 = true;
//                }

            telemetry.addData("current pos", -(arm.getCurrentPosition() - bottomLimit));
            telemetry.addData("arm pos", armSupport.getCurrentPosition());
            telemetry.addData("SHARED HEIGHT", SHARED);
            telemetry.addData("gate", gate.getPosition());
            telemetry.addData("level", level);
            telemetry.addData("toggle2", toggle2);
            telemetry.addData("target level", currentTarget);
            telemetry.addData("time", (int) timer.seconds());
            telemetry.addData("gyro reading", imu.getAngularOrientation().thirdAngle);
            telemetry.update();
        }
    }
}
