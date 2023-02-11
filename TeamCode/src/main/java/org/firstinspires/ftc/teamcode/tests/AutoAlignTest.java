package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AlignThread;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
@Config
public class AutoAlignTest extends LinearOpMode {
    public static double cameraPoint = .75;
    public void runOpMode(){
//        DcMotor bl, br, fl, fr;
        DistanceSensor backDist, frontDist;
        double distance;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
        AlignThread aligner = new AlignThread(hardwareMap);
        Thread alignerThread = new Thread(aligner);

//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        Rev2mDistanceSensor dist = (Rev2mDistanceSensor) backDist;

//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);

//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted() && !isStopRequested()) {
            pipeline.setPipelines("pole", "pole");
            pipeline.frontPoleDetector.setColors(true, false, false);
            pipeline.backPoleDetector.setColors(true, false, false);
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        alignerThread.start();
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
            telemetry.addData("width", pipeline.getMaxWidth(false));
            telemetry.addData("height", pipeline.getMaxHeight(false));
            telemetry.addData("distance", (int)(382.3333333 / pipeline.getMaxWidth(false)));
            telemetry.addData("calculated dist", aligner.getRobotDistance(true));
            telemetry.addData("angle", aligner.getAngle(true));
            telemetry.addData("xDist", aligner.xDist);
            telemetry.addData("yDist", aligner.yDist);
            telemetry.update();
        }
        alignerThread.interrupt();
    }
}
