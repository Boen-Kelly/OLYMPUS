package org.firstinspires.ftc.teamcode.oldcode.TeleOp;

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
@TeleOp(name = "Demo Op Mode")
@Disabled
public class DemoOpMode extends LinearOpMode {
    public static double gateClosed = .1, gateOpen = .37, bottomClosed = 1, bottomOpen = 0, quasiGateOpen = .2, straight, strafe, rotation;
    public static int SHARED = 175, BOTTOM = 175, MIDDLE = 250, HIGH = 350,  targetPos = 0;

    public void runOpMode() {

        DcMotor intake, arm, armSupport;
        DcMotorEx duckWheel;
        Servo gate, bottom;
        DigitalChannel limit;
        BNO055IMU imu;
        double slowDown = 1, armPower = 1;
        int bottomLimit = 0, level = 0, levelHeight = 350;
        boolean toggle = false, armState = false, toggle2 = false, toggle3 = false, toggle4 = false, activatedArm = true;
        boolean toggle5 = false, armOn = true;
        String currentTarget = "top";
        TapeMeasure tape = new TapeMeasure(hardwareMap, gamepad1, gamepad2, TapeMeasure.Side.RED);
        Thread tapeThread = new Thread(tape);

        Gamepad.RumbleEffect fastBlip;

        fastBlip = new Gamepad.RumbleEffect.Builder()
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .build();

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

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        armSupport.setDirection(DcMotorSimple.Direction.REVERSE);

        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSupport.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limit.setMode(DigitalChannel.Mode.INPUT);

        gate.scaleRange(0,1);
        bottom.scaleRange(0,1);

        imu.initialize(parameters);

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            telemetry.addLine("not initialized");
            telemetry.update();
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        tapeThread.start();
        while (opModeIsActive()) {

            if(!tape.isControlsActive()) {
                if (gamepad1.dpad_right) {
                    gamepad1.runRumbleEffect(fastBlip);
                }
                if (gamepad2.x) {
                    //TODO: I think we should investigate this further in the future, but as for now,
                    // competition is next week.
//                int startPos = duckWheel.getCurrentPosition();
//                duckWheel.setVelocity(180, AngleUnit.DEGREES);
//                if(duckWheel.getCurrentPosition() + startPos > 400){
//                    duckWheel.setVelocity(360, AngleUnit.DEGREES);
//                }
                    duckWheel.setVelocity(180, AngleUnit.DEGREES);
                } else if (gamepad2.b) {
                    duckWheel.setVelocity(-180, AngleUnit.DEGREES);
                } else {
                    duckWheel.setPower(0);
                }

                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }

                if (!limit.getState()) {
                    bottomLimit = arm.getCurrentPosition();
                }

                if (arm.getCurrentPosition() <= -400 && !(gamepad1.left_trigger > 0) && !(imu.getAngularOrientation().thirdAngle < (90 - 15))) {
                    arm.setTargetPosition(bottomLimit - 450);
                    armSupport.setTargetPosition(arm.getTargetPosition());
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armSupport.setMode(arm.getMode());
                    arm.setPower(armPower);
                    armSupport.setPower(arm.getPower());
                } else if (gamepad1.left_trigger > 0 && limit.getState()) {
                    armPower = .5;
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setPower(gamepad1.left_trigger * armPower);
                    armSupport.setPower(arm.getPower());
                    targetPos = arm.getCurrentPosition();
                } else if (gamepad1.right_trigger > 0 && arm.getCurrentPosition() > bottomLimit - 400) {
                    armPower = .5;
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setPower(-gamepad1.right_trigger * armPower);
                    armSupport.setPower(arm.getPower());
                    targetPos = arm.getCurrentPosition();
                } else {
                    if (armOn) {
                        arm.setTargetPosition(targetPos);
                        armSupport.setTargetPosition(arm.getTargetPosition());
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armSupport.setMode(arm.getMode());
                        arm.setPower(.5);
                        armSupport.setPower(arm.getPower());
                    } else {
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        arm.setPower(0);
                        armSupport.setPower(0);
                    }
                }

                if (gamepad1.left_stick_button) {
                    if (toggle5) {
                        armOn = !armOn;
                        toggle5 = false;
                    }
                } else {
                    toggle5 = true;
                }

                if (gamepad1.dpad_up) {
                    if (toggle2) {
                        level++;
                        toggle2 = false;
                    }
                } else if (gamepad1.dpad_down) {
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

                if (gamepad2.right_bumper) {
                    if (toggle3) {
                        SHARED += 10;
                        toggle3 = false;
                    }
                } else if (gamepad2.left_bumper) {
                    if (toggle3) {
                        SHARED -= 10;
                        toggle3 = false;
                    }
                } else if (gamepad2.right_trigger > 0) {
                    if (toggle3) {
                        SHARED += 20;
                        toggle3 = false;
                    }
                } else if (gamepad2.left_trigger > 0) {
                    if (toggle3) {
                        SHARED -= 20;
                        toggle3 = false;
                    }
                } else {
                    toggle3 = true;
                }

                if (gamepad1.a && !(gamepad1.options || gamepad2.options)) {
                    if (toggle) {
                        armState = !armState;
                        toggle = false;
                    }
                } else {
                    toggle = true;
                }

                if (gamepad1.a && armState) {
                    targetPos = bottomLimit - levelHeight;
                } else if (gamepad1.a && !armState) {
                    targetPos = bottomLimit;
                }

                if (gamepad1.dpad_left) {
                    if (toggle4) {
                        activatedArm = !activatedArm;
                        toggle4 = false;
                    }
                } else {
                    toggle4 = true;
                }

                if ((imu.getAngularOrientation().thirdAngle < (90 - 15)) && activatedArm) {
                    targetPos = bottomLimit - 100;
                    armState = true;
                }

                if ((arm.getCurrentPosition() <= bottomLimit - 300) && gamepad1.y) {
                    gate.setPosition(gateOpen);

                    if (gamepad1.b) {
                        bottom.setPosition(bottomOpen);
                    } else {
                        bottom.setPosition(bottomClosed);
                    }
                } else if (arm.getCurrentPosition() >= bottomLimit - 100 && (gamepad1.right_bumper || gamepad1.y)) {
                    gate.setPosition(gateOpen);

                    if (gamepad1.b) {
                        bottom.setPosition(bottomOpen);
                    } else {
                        bottom.setPosition(bottomClosed);
                    }
                } else if (bottomLimit - 100 > arm.getCurrentPosition() && arm.getCurrentPosition() > bottomLimit - 300 && gamepad1.y) {
                    bottom.setPosition(bottomOpen);

                    if (gamepad1.x) {
                        gate.setPosition(gateOpen);
                    } else {
                        gate.setPosition(quasiGateOpen);
                    }
                } else {
                    if (gamepad1.b) {
                        bottom.setPosition(bottomOpen);
                    } else {
                        bottom.setPosition(bottomClosed);
                    }

                    if (gamepad1.x) {
                        gate.setPosition(gateOpen);
                    } else {
                        gate.setPosition(gateClosed);
                    }
                }

//            telemetry.addData("target pos", targetPos);
                telemetry.addData("current pos", -(arm.getCurrentPosition() - bottomLimit));
                telemetry.addData("arm pos", armSupport.getCurrentPosition());
//            telemetry.addData("left stick y", gamepad1.left_stick_y);
//            telemetry.addData("motor power", intake.getPower());
//            telemetry.addData("duck wheel", duckWheel.getPower());
//            telemetry.addData("touch sensor", limit.getState());
//            telemetry.addData("arm power", arm.getPower());
//            telemetry.addData("armSupport power", arm.getPower());
//            telemetry.addData("armState", armState);
                telemetry.addData("SHARED HEIGHT", SHARED);
                telemetry.addData("gate", gate.getPosition());
                telemetry.addData("level", level);
                telemetry.addData("toggle2", toggle2);
                telemetry.addData("target level", currentTarget);
                telemetry.addLine("ARM WILL " + (activatedArm ? "OPEN WHEN TILTED" : "NOT OPEN WHEN TILTED"));
                telemetry.update();
            }
        }
        tapeThread.interrupt();
    }
}
