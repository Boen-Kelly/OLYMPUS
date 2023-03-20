package org.firstinspires.ftc.teamcode.Autos.BLUE;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.classes.AlignThread;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;
import org.firstinspires.ftc.teamcode.classes.lightsThread;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "5 cone auto")
@Config
public class VisionFiveConeAuto extends LinearOpMode{
    public static double exposure = 10;
    public static int gain = 25;
    public static int WB = 6000;
    public void runOpMode() throws InterruptedException{
        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
        int AprilTagID =7;

        //Tag IDs
        int left = 6;
        int middle = 7;
        int right = 8;

        boolean toggle1 = false;
        boolean toggle2 = false;

        double alignStartTime = 0;
        double alignedTime = 0;

        double straight = 0, strafe = 0, rotate = 0;
        Pose2d startingPose;
        Pose2d currentPose = new Pose2d();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
        AlignThread aligner = new AlignThread(hardwareMap, pipeline, parameters);
        Thread alignerThread = new Thread(aligner);

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);

        DistanceSensor backDist, frontDist, sideDist;
        Servo lilArmL, lilArmR;
        DcMotor bl, br, fl, fr;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        lilArmL = hardwareMap.get(Servo.class, "lilArm1");
        lilArmR = hardwareMap.get(Servo.class, "lilArm2");
        sideDist = hardwareMap.get(DistanceSensor.class, "sideDist");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        LiftArm lift = new LiftArm(hardwareMap);

        String[] camData = new String[3];
        int c = 0;

        try {
            File obj = new File("/sdcard/FIRST/camVals.txt");
            Scanner scan = new Scanner(obj);

            while (scan.hasNextLine()){
                telemetry.addLine("Reading...");
                telemetry.update();
                camData[c] = scan.nextLine();
                c ++;
            }
            scan.close();
        }catch (FileNotFoundException e){
            telemetry.addLine("couldn't read");
            telemetry.update();
            e.printStackTrace();
        }

        exposure = Double.parseDouble(camData[0]);
        gain = Integer.parseInt(camData[1]);
        WB = Integer.parseInt(camData[2]);

        lilArmL.setPosition(.047);
        lilArmR.setPosition(.8);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 64.75, Math.toRadians(90)));
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime timer = new ElapsedTime();

        alignerThread.start();

        while(!alignerThread.isAlive()){
            telemetry.addLine("Creating aligner thread");
            telemetry.update();
        }

        TrajectorySequence pushCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () ->{
                    lift.setSlurpPower(1);
                    pipeline.setPipelines("pole", "pole");
                    aligner.camera.pointCam(.6,.6);
                })
                .addTemporalMarker(2, () -> {
                    lift.lift(2200, true);
                })
                .lineToLinearHeading(new Pose2d(-36, 3, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        pipeline.setPipelines("pole", "sleeve");
        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);
        aligner.camera.pointCam(.6, .55);

        while(!isStarted() && !isStopRequested()) {
            if(gamepad1.a){
                pipeline.setPipelines("pole", "pole");
                pipeline.frontPoleDetector.setColors(true, false, false);
                pipeline.backPoleDetector.setColors(true, false, false);
                aligner.camera.pointCam(.6, .4);
            }else if(gamepad1.b){
                pipeline.setPipelines("sleeve", "sleeve");
                aligner.camera.pointCam(.6, .6);
            }else if(gamepad1.y){
                pipeline.setPipelines("pole", "sleeve");
                pipeline.backPoleDetector.setColors(true, false, false);
                pipeline.frontPoleDetector.setColors(false, false, true);
                aligner.camera.pointCam(.6, .55);
            }

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

            telemetry.addData("Sleeve position", AprilTagID);
            telemetry.addData("exposure", exposure);
            telemetry.addData("gain", gain);
            telemetry.addData("WB", WB);
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        pipeline.setPipelines("pole", "sleeve");
        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);
        aligner.camera.pointCam(.6, .55);

        waitForStart();
        timer.reset();

        drive.followTrajectorySequence(pushCone);

        aligner.camera.enableCam(false);
        aligner.engageMaster(6, false, 135, true);

        startingPose = drive.getPoseEstimate();

        double AlignerTime = timer.time(TimeUnit.MILLISECONDS);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(alignedTime < 250 /** && (timer.time(TimeUnit.MILLISECONDS) - AlignerTime) < 3000*/) {
            aligner.updateHardware(Math.toDegrees(drive.getPoseEstimate().getHeading()),frontDist.getDistance(DistanceUnit.INCH), backDist.getDistance(DistanceUnit.INCH));

            currentPose = drive.getPoseEstimate();

            fl.setPower(aligner.rotate - strafe - straight);
            fr.setPower(-aligner.rotate + strafe - straight);
            bl.setPower(aligner.rotate + strafe - straight);
            br.setPower(-aligner.rotate - strafe - straight);

            strafe = aligner.strafe;
            straight = aligner.straight;

            if(aligner.aligned()){
                if(toggle2){
                    alignStartTime = timer.time(TimeUnit.MILLISECONDS);
                    toggle2 = false;
                }
                alignedTime = timer.time(TimeUnit.MILLISECONDS) - alignStartTime;
            }else{
                alignedTime = 0;
                toggle2 = true;
            }

            if(aligner.camera.getAngle(false) < 10 || sideDist.getDistance(DistanceUnit.INCH) < 6){
                break;
            }

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -straight,
//                            strafe,
//                            -aligner.rotate
//                    )
//            );

            telemetry.addData("cam angle", aligner.camera.getAngle(false));
            telemetry.addData("robot angle", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("xDist", (int)aligner.xDist);
            telemetry.addData("yDist", (int)aligner.yDist);
            telemetry.addData("back dist", backDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("aligned?", aligner.aligned());
            telemetry.addData("x?", aligner.xDirection());
            telemetry.addData("y?", aligner.yDirection());
            telemetry.addData("theta?", aligner.heading());
            telemetry.addData("y speed", aligner.straight);
            telemetry.addData("x speed", aligner.strafe);
            telemetry.addData("alignedTime", alignedTime);
            telemetry.addData("drive x", currentPose.getX());
            telemetry.addData("drive y", currentPose.getY());
            telemetry.update();
            drive.update();
        }

        telemetry.addLine("DONE!");
        telemetry.addData("liftThread", lift);
        telemetry.update();

        aligner.camera.disableCam(false);
        aligner.disengageMaster();
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        drive.update();

        lift.lift(1600, true);

        lift.setSlurpPower(-1);
        sleep(500);

        for (int i = 0; i < 3; i++) {
            TrajectorySequence toStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1000, false);
                        lift.setSlurpPower(1);
                        aligner.camera.pointCam(.7, .6);
                    })
                    .setTangent(Math.toRadians(135))
                    .splineToSplineHeading(new Pose2d(-48,12, Math.toRadians(180)),Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180))
                    .build();

            drive.followTrajectorySequence(toStack);

            aligner.camera.enableCam(true);
            aligner.engageMaster(0, true, 180, false);

            startingPose = drive.getPoseEstimate();

            alignedTime = 0;

            AlignerTime = timer.time(TimeUnit.MILLISECONDS);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (alignedTime < 10/** && (timer.time(TimeUnit.MILLISECONDS) - AlignerTime) < 3000*/) {
                aligner.updateHardware(Math.toDegrees(drive.getPoseEstimate().getHeading()), frontDist.getDistance(DistanceUnit.INCH), backDist.getDistance(DistanceUnit.INCH));

//                currentPose = drive.getPoseEstimate();

                fl.setPower(aligner.rotate + aligner.strafe + aligner.straight);
                fr.setPower(-aligner.rotate - aligner.strafe + aligner.straight);
                bl.setPower(aligner.rotate - aligner.strafe + aligner.straight);
                br.setPower(-aligner.rotate + aligner.strafe + aligner.straight);

//                if(Math.abs(startingPose.getX() - currentPose.getX()) > 10){
//                    strafe = 0;
//                }else{
//                    strafe = aligner.strafe;
//                }
//
//                if(Math.abs(startingPose.getY() - currentPose.getY()) > 20){
//                    straight = 0;
//                    break;
//                }else{
//                    straight = aligner.straight;
//                }

                if(aligner.aligned()){
                    if(toggle2){
                        alignStartTime = timer.time(TimeUnit.MILLISECONDS);
                        toggle2 = false;
                    }
                    alignedTime = timer.time(TimeUnit.MILLISECONDS) - alignStartTime;
                }else{
                    alignedTime = 0;
                    toggle2 = true;
                }

//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                straight,
//                                -strafe,
//                                -aligner.rotate
//                        )
//                );

                telemetry.addData("cam angle", aligner.camera.getAngle(false));
                telemetry.addData("robot angle", Math.toDegrees(drive.getPoseEstimate().getHeading()));
                telemetry.addData("xDist", (int)aligner.xDist);
                telemetry.addData("yDist", (int)aligner.yDist);
                telemetry.addData("back dist", backDist.getDistance(DistanceUnit.INCH));
                telemetry.addData("Thread back dist", aligner.backDist);
                telemetry.addData("aligned?", aligner.aligned());
                telemetry.addData("x speed", aligner.strafe);
                telemetry.addData("y speed", aligner.straight);
                telemetry.addData("rotate", aligner.rotate);
                telemetry.addData("drive x", currentPose.getX());
                telemetry.addData("drive y", currentPose.getY());
                telemetry.update();
                drive.update();
            }

            telemetry.addLine("DONE!");
            telemetry.update();

            aligner.camera.disableCam(true);
            aligner.disengageMaster();

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            drive.update();

            if(i == 4){
                lift.lift(0, false);
            }else {
                lift.lift(350 - i * 200, false);
            }
            sleep(500);
            lift.lift(1000, false);

            TrajectorySequence toDeliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
                        lift.lift(500,false);
                        aligner.camera.pointCam(.5, .6);
                    })
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-45,12),Math.toRadians(0))
                    .addTemporalMarker(1.5, () -> {
                        lift.lift(2200, true);
                    })
                    .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(135)), Math.toRadians(0))
                    .build();

            drive.followTrajectorySequence(toDeliver);

            aligner.camera.enableCam(false);
            aligner.engageMaster(5, false, 135, true);

            startingPose = drive.getPoseEstimate();

            alignedTime = 0;

            AlignerTime = timer.time(TimeUnit.MILLISECONDS);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(alignedTime < 250 /** && (timer.time(TimeUnit.MILLISECONDS) - AlignerTime) < 3000*/) {
                aligner.updateHardware(Math.toDegrees(drive.getPoseEstimate().getHeading()),frontDist.getDistance(DistanceUnit.INCH), backDist.getDistance(DistanceUnit.INCH));

                currentPose = drive.getPoseEstimate();

                fl.setPower(aligner.rotate - strafe - straight);
                fr.setPower(-aligner.rotate + strafe - straight);
                bl.setPower(aligner.rotate + strafe - straight);
                br.setPower(-aligner.rotate - strafe - straight);

                strafe = aligner.strafe;

                straight = aligner.straight;

                if(aligner.aligned()){
                    if(toggle2){
                        alignStartTime = timer.time(TimeUnit.MILLISECONDS);
                        toggle2 = false;
                    }
                    alignedTime = timer.time(TimeUnit.MILLISECONDS) - alignStartTime;
                }else{
                    alignedTime = 0;
                    toggle2 = true;
                }

                if(aligner.camera.getAngle(false) < 10 || sideDist.getDistance(DistanceUnit.INCH) < 6){
                    break;
                }

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -straight,
//                            strafe,
//                            -aligner.rotate
//                    )
//            );

                telemetry.addData("cam angle", aligner.camera.getAngle(false));
                telemetry.addData("robot angle", Math.toDegrees(drive.getPoseEstimate().getHeading()));
                telemetry.addData("xDist", (int)aligner.xDist);
                telemetry.addData("yDist", (int)aligner.yDist);
                telemetry.addData("back dist", backDist.getDistance(DistanceUnit.INCH));
                telemetry.addData("aligned?", aligner.aligned());
                telemetry.addData("straight", aligner.straight);
                telemetry.addData("strafe", aligner.strafe);
                telemetry.addData("rotate", aligner.rotate);
                telemetry.addData("drive x", currentPose.getX());
                telemetry.addData("drive y", currentPose.getY());
                telemetry.update();
                drive.update();
            }

            telemetry.addLine("DONE!");
            telemetry.addData("liftThread", lift);
            telemetry.update();

            aligner.camera.disableCam(false);
            aligner.disengageMaster();
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            drive.update();

            lift.lift(1600, true);

            lift.setSlurpPower(-1);
            sleep(500);
        }

        if(AprilTagID == left) {
            Trajectory parkL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.lift(0, false);
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .splineTo(new Vector2d(-36, 12), Math.toRadians(0),
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_ACCEL, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(new Vector2d(-24, 12), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-12, 12), Math.toRadians(0))
                    .build();

            drive.followTrajectory(parkL);
        }else if(AprilTagID == middle) {
            Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.lift(0, false);
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .build();

            drive.followTrajectory(park);
        }else if(AprilTagID == right) {
            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.lift(0, false);
                    })
                    .lineToLinearHeading(new Pose2d(-34,12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-60,12, Math.toRadians(0)))
                    .build();

            drive.followTrajectorySequence(park);
        }

        try {
            File test = new File("/sdcard/FIRST/nums.txt");
            File camVals = new File("/sdcard/FIRST/camVals.txt");

            FileWriter writer = new FileWriter("/sdcard/FIRST/nums.txt");
            FileWriter camValWriter = new FileWriter("/sdcard/FIRST/camVals.txt");

            writer.write(Math.toDegrees(drive.getPoseEstimate().getHeading()) + "");
            camValWriter.write(exposure + "\n" + gain + "\n" + WB);

            writer.close();
            camValWriter.close();

            telemetry.addLine("successfully wrote!");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("couldn't create file");
            telemetry.update();
            e.printStackTrace();
        }

        double endingTime = timer.time(TimeUnit.SECONDS);
        while(opModeIsActive()){
            telemetry.addData("ending time", endingTime);
            telemetry.update();
        }
        aligner.stopCamera();
        alignerThread.interrupt();
    }
}