//package org.firstinspires.ftc.teamcode.Autos.Tests;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.classes.AlignThread;
//import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
//import org.firstinspires.ftc.teamcode.classes.LiftArm;
//import org.firstinspires.ftc.teamcode.classes.lightsThread;
//
//import java.io.File;
//import java.io.FileNotFoundException;
//import java.util.Scanner;
//import java.util.concurrent.TimeUnit;
//
//@TeleOp(name = "5 cone auto")
//@Config
//public class VisionFiveConeAuto extends LinearOpMode{
//    public static double exposure = 10;
//    public static int gain = 25;
//    public static int WB = 6000;
//    public void runOpMode() {
//        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
//        int AprilTagID =7;
//
//        //Tag IDs
//        int left = 6;
//        int middle = 7;
//        int right = 8;
//
//        boolean toggle1 = false;
//
//        double straight = 0, strafe = 0, rotate = 0;
//        Pose2d startingPose;
//        Pose2d currentPose = new Pose2d();
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
//
//        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
//        AlignThread aligner = new AlignThread(hardwareMap, pipeline, parameters);
//        Thread alignerThread = new Thread(aligner);
//
//        while(!pipeline.toString().equals("waiting for start")){
//            telemetry.addLine("waiting for OpenCV");
//            telemetry.update();
//        }
//
//        pipeline.backPoleDetector.setColors(true, false, false);
//        pipeline.frontPoleDetector.setColors(false, false, true);
//
//        DistanceSensor backDist, frontDist;
//        Servo lilArmL, lilArmR;
//        DcMotor bl, br, fl, fr;
//
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
//        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
//        lilArmL = hardwareMap.get(Servo.class, "lilArm1");
//        lilArmR = hardwareMap.get(Servo.class, "lilArm2");
//
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        LiftArm lift = new LiftArm(hardwareMap);
//
//        String[] camData = new String[3];
//        int c = 0;
//
//        try {
//            File obj = new File("/sdcard/FIRST/camVals.txt");
//            Scanner scan = new Scanner(obj);
//
//            while (scan.hasNextLine()){
//                telemetry.addLine("Reading...");
//                telemetry.update();
//                camData[c] = scan.nextLine();
//                c ++;
//            }
//            scan.close();
//        }catch (FileNotFoundException e){
//            telemetry.addLine("couldn't read");
//            telemetry.update();
//            e.printStackTrace();
//        }
//
//        exposure = Double.parseDouble(camData[0]);
//        gain = Integer.parseInt(camData[1]);
//        WB = Integer.parseInt(camData[2]);
//
//        lilArmL.setPosition(.047);
//        lilArmR.setPosition(.8);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(-36, 64.75, Math.toRadians(90)));
//        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        ElapsedTime timer = new ElapsedTime();
//
//        alignerThread.start();
//
//        while(!alignerThread.isAlive()){
//            telemetry.addLine("Creating aligner thread");
//            telemetry.update();
//        }
//
//        TrajectorySequence pushCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .addTemporalMarker(() ->{
//                    lift.setSlurpPower(1);
//                    lift.lift(0,true);
//                    pipeline.setPipelines("pole", "pole");
//                })
//                .lineToConstantHeading(new Vector2d(-36,-0.4))
//                .build();
//        TrajectorySequence firstDeliver = drive.trajectorySequenceBuilder(pushCone.end())
//                .addTemporalMarker(() -> {
//                    lift.lift(1850, true);
//                })
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(-31,7, Math.toRadians(135)),Math.toRadians(-45))
//                .build();
//        TrajectorySequence toStack = drive.trajectorySequenceBuilder(firstDeliver.end())
//                .setTangent(Math.toRadians(135))
//                .splineToSplineHeading(new Pose2d(-48,12, Math.toRadians(180)),Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-65, 12), Math.toRadians(180))
//                .build();
//        TrajectorySequence toDeliver = drive.trajectorySequenceBuilder(toStack.end())
//                .setTangent(Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-48,12),Math.toRadians(0))
//                .addDisplacementMarker(5, () -> {
//                    liftUp(1850, true);
//                })
//                .splineToSplineHeading(new Pose2d(-26.2, 2.2, Math.toRadians(135)), Math.toRadians(315))
//                .build();
//
//        while(!isStarted() && !isStopRequested()) {
//            if(gamepad1.a){
//                pipeline.setPipelines("pole", "pole");
//                pipeline.frontPoleDetector.setColors(true, false, false);
//                pipeline.backPoleDetector.setColors(true, false, false);
//                aligner.camera.pointCam(.6, .4);
//            }else if(gamepad1.b){
//                pipeline.setPipelines("sleeve", "sleeve");
//                aligner.camera.pointCam(.6, .6);
//            }else if(gamepad1.y){
//                pipeline.setPipelines("sleeve", "pole");
//                pipeline.backPoleDetector.setColors(true, false, false);
//                pipeline.frontPoleDetector.setColors(false, false, true);
//                aligner.camera.pointCam(.6, .6);
//            }
//
//            if(gamepad1.right_bumper){
//                if(toggle1){
//                    exposure += 5;
//                    toggle1 = false;
//                }
//            }else if(gamepad1.left_bumper){
//                if(toggle1){
//                    exposure -= 5;
//                    toggle1 = false;
//                }
//            }else if(gamepad1.dpad_up){
//                if(toggle1){
//                    gain += 10;
//                    toggle1 = false;
//                }
//            }else if(gamepad1.dpad_down){
//                if(toggle1){
//                    gain -= 10;
//                    toggle1 = false;
//                }
//            }else if(gamepad1.dpad_right){
//                if(toggle1){
//                    WB += 500;
//                    toggle1 = false;
//                }
//            }else if(gamepad1.dpad_left){
//                if(toggle1) {
//                    WB -= 500;
//                    toggle1 = false;
//                }
//            }else{
//                toggle1 = true;
//            }
//
//            exposure = Math.max(1,exposure);
//            gain = Math.max(1, gain);
//            WB = Math.max(1, WB);
//
//            pipeline.setCamVals(exposure,gain,WB);
//
//            AprilTagID = pipeline.AprilTagID(true);
//
//            telemetry.addData("Sleeve position", AprilTagID);
//            telemetry.addData("exposure", exposure);
//            telemetry.addData("gain", gain);
//            telemetry.addData("WB", WB);
//            telemetry.addLine("waiting for start");
//            telemetry.update();
//        }
//
//        pipeline.setPipelines("sleeve", "pole");
//        pipeline.backPoleDetector.setColors(true, false, false);
//        pipeline.frontPoleDetector.setColors(false, false, true);
//        aligner.camera.pointCam(.6, .6);
//
//        waitForStart();
//        timer.reset();
//
//        drive.followTrajectorySequence(pushCone);
//        drive.followTrajectorySequence(firstDeliver);
//
//        aligner.camera.enableCam(false);
//        aligner.engageMaster(2, false, 135, true);
//
//        startingPose = drive.getPoseEstimate();
//
//        double AlignerTime = timer.time(TimeUnit.MILLISECONDS);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while(!aligner.aligned()) {
//            aligner.updateHardware(Math.toDegrees(drive.getPoseEstimate().getHeading()),frontDist.getDistance(DistanceUnit.INCH), backDist.getDistance(DistanceUnit.INCH));
//
//            currentPose = drive.getPoseEstimate();
//
//            fl.setPower(aligner.rotate - aligner.strafe - aligner.straight);
//            fr.setPower(-aligner.rotate + aligner.strafe - aligner.straight);
//            bl.setPower(aligner.rotate + aligner.strafe - aligner.straight);
//            br.setPower(-aligner.rotate - aligner.strafe - aligner.straight);
//
//            if(Math.abs(startingPose.getX() - currentPose.getX()) > 10){
//                strafe = 0;
//            }else{
//                strafe = aligner.strafe;
//            }
//
//            if(Math.abs(startingPose.getY() - currentPose.getY()) > 20){
//                straight = 0;
//                break;
//            }else{
//                straight = aligner.straight;
//            }
//
//            drive.update();
//        }
//
//        telemetry.addLine("DONE!");
//        telemetry.addData("liftThread", lift);
//        telemetry.update();
//
//        aligner.camera.disableCam(false);
//        aligner.disengageMaster();
//        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//
//        drive.update();
//
//        lift.lift(1650, true);
//
//        lift.setSlurpPower(-1);
//
//        for (int i = 0; i < 5; i++) {
//            liftUp(1000, false);
//            slurper.setPower(1);
//            drive.followTrajectorySequence(toStack);
//            liftUp(500, false);
//            sleep(500);
//            liftUp(1000, false);
//            drive.followTrajectorySequence(toDeliver);
//            slurper.setPower(-1);
//            sleep(500);
//        }
//
//        liftDown();
//        sleep(1000);
//    }
//
//    public void liftUp(int height, boolean armOver){
//        if(armOver){
//            arm.setTargetPosition(-1310);
//        }else{
//            arm.setTargetPosition(0);
//        }
//        lift1.setTargetPosition(height);
//        lift2.setTargetPosition(lift1.getTargetPosition());
//        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift2.setMode(lift1.getMode());
//        lift1.setPower(1);
//        lift2.setPower(lift1.getPower());
//    }
//    public void liftDown(){
//        arm.setTargetPosition(0);
//        lift1.setTargetPosition(0);
//        lift2.setTargetPosition(lift1.getTargetPosition());
//        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift2.setMode(lift1.getMode());
//        lift1.setPower(1);
//        lift2.setPower(lift1.getPower());
//    }
//}