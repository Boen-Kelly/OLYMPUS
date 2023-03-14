package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.AlignThread;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

@TeleOp
//@Disabled
@Config
public class AutoAlignTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static double frontPoint = .75;
    public static double backPoint = .5;
    public static double exposure = 10;
    public static int gain = 1;
    public static double distance = 3;
    public static int WB = 5000;
    public static boolean AEPriority = false;
    boolean toggle1 = false;
    double lastTime = 0;
    double packetSentTime = 0;

    public void runOpMode(){
        DcMotor bl, br, fl, fr;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
//            drive.update();
            pipeline.setCamVals(exposure,gain,WB);

//            fl.setPower(aligner.rotate - aligner.strafe - aligner.straight);
//            fr.setPower(-aligner.rotate + aligner.strafe - aligner.straight);
//            bl.setPower(aligner.rotate + aligner.strafe - aligner.straight);
//            br.setPower(-aligner.rotate - aligner.strafe - aligner.straight);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -aligner.straight,
                            aligner.strafe,
                            -aligner.rotate
                    )
            );

            if(gamepad1.a){
                aligner.camera.enableCam(false);
                aligner.engageMaster(distance,false, 45);
            }else if(gamepad1.y || aligner.aligned()){
                aligner.camera.disableCam(false);
                aligner.disengageMaster();
            }else if(gamepad1.x){
                aligner.camera.enableCam(false);
            }else if(gamepad1.b){
                aligner.camera.disableCam(false);
                aligner.camera.pointCam(frontPoint, backPoint);
            }

//            telemetry.addData("Pipeline says", pipeline);
            telemetry.addData("calculated dist", aligner.getRobotDistance(false));
            telemetry.addData("angle", aligner.camera.getAngle(false));
            telemetry.addData("xDist", (int)aligner.xDist);
            telemetry.addData("yDist", (int)aligner.yDist);
//            telemetry.addData("aligned?", aligner.aligned());
//            telemetry.addData("gyro angle", aligner.gyroHeading());
//            telemetry.addData("backDist", aligner.backDist.getDistance(DistanceUnit.INCH));
//            telemetry.addData("frontDist", aligner.frontDist.getDistance(DistanceUnit.INCH));
//            telemetry.addData("drive x", drive.getPoseEstimate().getX());
//            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("exposure", exposure);
            telemetry.addData("gain", gain);
            telemetry.addData("WB", WB);
            telemetry.addData("AlignTest loop time", timer.time() - lastTime);
            telemetry.addData("AlignThread loop time", aligner.getCycleTime());
            telemetry.addData("straight", aligner.straight);
            telemetry.addData("strafe", aligner.strafe);
            telemetry.addData("rotate", aligner.rotate);
            telemetry.update();

            drive.update();

//            if((timer.time() - packetSentTime) > 20) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay()
                        .setFill("goldenrod")
                        .fillCircle(aligner.yDist, aligner.xDist, 2);
                dashboard.sendTelemetryPacket(packet);
                packetSentTime = timer.time();
//            }

            lastTime = timer.time();
        }
        alignerThread.interrupt();
    }
}
