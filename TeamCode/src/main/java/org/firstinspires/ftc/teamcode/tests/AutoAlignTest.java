package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AlignThread;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
@Config
public class AutoAlignTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static double cameraPoint = .75;
    public static double exposure = 25;
    public static int gain = 1;
    public static int WB = 5000;
    public static boolean AEPriority = false;
    boolean toggle1 = false;
    double lastTime = 0;

    public void runOpMode(){
//        DcMotor bl, br, fl, fr;
        double distance;

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry telemetry = dashboard.getTelemetry();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

        telemetry.addLine("pipeline");
        telemetry.update();
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
        telemetry.addLine("aligner");
        telemetry.update();
        AlignThread aligner = new AlignThread(hardwareMap, pipeline, parameters);
        telemetry.addLine("thread");
        telemetry.update();
        Thread alignerThread = new Thread(aligner);


//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");

//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);

//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        alignerThread.start();

        while(!isStarted() && !isStopRequested()) {
            pipeline.setPipelines("pole", "pole");
            pipeline.frontPoleDetector.setColors(true, false, false);
            pipeline.backPoleDetector.setColors(true, false, false);

            telemetry.addLine("Waiting for start");
            telemetry.update();
        }

        timer.reset();
        while (opModeIsActive()){

            if(gamepad1.right_bumper){
                if(toggle1){
                    exposure += 10;
                    toggle1 = false;
                }
            }else if(gamepad1.left_bumper){
                if(toggle1){
                    exposure -= 10;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_up){
                if(toggle1){
                    gain += 1;
                    toggle1 = false;
                }
            }else if(gamepad1.dpad_down){
                if(toggle1){
                    gain -= 1;
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

            pipeline.setCamVals(exposure,gain,WB);

//            fl.setPower(pipeline.align(cameraPoint, .1, false));
//            br.setPower(-pipeline.align(cameraPoint, .1, false));
//            bl.setPower(pipeline.align(cameraPoint, .1, false));
//            fr.setPower(-pipeline.align(cameraPoint, .1, false));

//            fl.setPower(pipeline.align(cameraPoint, .3, true));
//            br.setPower(-pipeline.align(cameraPoint, .3, true));
//            bl.setPower(pipeline.align(cameraPoint, .3, true));
//            fr.setPower(-pipeline.align(cameraPoint, .3 , true));

//            pipeline.aimCam();

            if(gamepad1.a){
                aligner.engageMaster(5,true, 0);
            }else if(gamepad1.y || aligner.aligned()){
                aligner.disengageMaster();
            }

//            telemetry.addData("Pipeline says", pipeline);
//            telemetry.addData("calculated dist", aligner.getRobotDistance(true));
//            telemetry.addData("angle", aligner.getAngle(true));
//            telemetry.addData("xDist", (int)aligner.xDist);
//            telemetry.addData("yDist", (int)aligner.yDist);
//            telemetry.addData("aligned?", aligner.aligned());
//            telemetry.addData("gyro angle", aligner.gyroHeading());
//            telemetry.addData("gyro speed", aligner.gyroAlign(0));
//            telemetry.addData("xSpeed", aligner.xSpeed);
//            telemetry.addData("ySpeed", aligner.ySpeed);
//            telemetry.addData("backDist", aligner.backDist.getDistance(DistanceUnit.INCH));
//            telemetry.addData("frontDist", aligner.frontDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("exposure", exposure);
            telemetry.addData("gain", gain);
            telemetry.addData("WB", WB);
            telemetry.addData("AlignTest loop time", timer.time() - lastTime);
            telemetry.addData("AlignThread loop time", aligner.getCycleTime());
            telemetry.update();

            lastTime = timer.time();
        }
        alignerThread.interrupt();
    }
}
