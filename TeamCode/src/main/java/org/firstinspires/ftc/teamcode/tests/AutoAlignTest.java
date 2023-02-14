package org.firstinspires.ftc.teamcode.tests;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.BNO055IMU;

import androidx.arch.core.executor.TaskExecutor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.oldcode.tests.classes.AlignThread;
import org.firstinspires.ftc.teamcode.oldcode.tests.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
@Config
public class AutoAlignTest extends LinearOpMode {
    public static double cameraPoint = .75;
    public static double exposure = 0;

    BNO055IMU imu;
    public void runOpMode(){
//        DcMotor bl, br, fl, fr;
        double distance;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("pipeline");
        telemetry.update();
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
        telemetry.addLine("aligner");
        telemetry.update();
        AlignThread aligner = new AlignThread(hardwareMap, pipeline);
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

        aligner.engageMaster(5, true);
        while (opModeIsActive()){

//            fl.setPower(pipeline.align(cameraPoint, .1, false));
//            br.setPower(-pipeline.align(cameraPoint, .1, false));
//            bl.setPower(pipeline.align(cameraPoint, .1, false));
//            fr.setPower(-pipeline.align(cameraPoint, .1, false));

//            fl.setPower(pipeline.align(cameraPoint, .3, true));
//            br.setPower(-pipeline.align(cameraPoint, .3, true));
//            bl.setPower(pipeline.align(cameraPoint, .3, true));
//            fr.setPower(-pipeline.align(cameraPoint, .3 , true));

//            pipeline.aimCam();


            telemetry.addData("Pipeline says", pipeline);
            telemetry.addData("distance", (int)(382.3333333 / pipeline.getMaxWidth(true)));
            telemetry.addData("calculated dist", aligner.getRobotDistance(true));
            telemetry.addData("angle", aligner.getAngle(true));
            telemetry.addData("xDist", (int)aligner.xDist);
            telemetry.addData("yDist", (int)aligner.yDist);
            telemetry.addData("gyro heading", imu.getAngularOrientation().firstAngle);
//            telemetry.addData("current exposure", pipeline.setExposure((long)exposure));
//            telemetry.addData("max exposure", pipeline.getMaxExposure(true));
            telemetry.update();
        }
        alignerThread.interrupt();
    }
}
