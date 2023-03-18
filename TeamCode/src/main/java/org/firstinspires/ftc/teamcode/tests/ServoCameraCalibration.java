package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
@Config
public class ServoCameraCalibration extends LinearOpMode {
    public static double frontPos = .82, backPos = .82;
    int AprilTagID =7;
    boolean toggle1;
    public static double exposure = 16;
    public static int WB = 6000, gain = 46;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo back, front;

        back = hardwareMap.get(Servo.class, "back");
        front = hardwareMap.get(Servo.class, "front");

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 1");

        front.scaleRange(0,.63);
        back.scaleRange(.22, .89);

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        while(!isStarted()) {
            pipeline.setPipelines("pole", "sleeve");
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            back.setPosition(backPos); //min pos is .28
            front.setPosition(frontPos);

            if(gamepad1.right_bumper){
                if(toggle1){
                    exposure += 5;
                    toggle1 = false;
                }
            }else if(gamepad1.left_bumper){
                if(toggle1){
                    exposure -= 5;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_up){
                if(toggle1){
                    gain += 10;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_down){
                if(toggle1){
                    gain -= 10;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_right){
                if(toggle1){
                    WB += 500;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_left){
                if(toggle1) {
                    WB -= 500;
                    toggle1 = false;
                }
            }else{
                toggle1 = true;
            }

            exposure = Math.max(1,exposure);
            gain = Math.max(1, gain);
            WB = Math.max(1, WB);

            pipeline.setCamVals(exposure,gain,WB);

            AprilTagID = pipeline.AprilTagID(false);

            telemetry.addData("angle", 180 - (front.getPosition() * 180) - 50.4);
            telemetry.addData("back angle", 180 - (180 * back.getPosition()));
            telemetry.addData("Sleeve position", AprilTagID);
            telemetry.update();
            //.72
        }
    }
}
