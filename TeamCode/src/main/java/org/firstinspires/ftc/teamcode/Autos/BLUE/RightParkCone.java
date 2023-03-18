package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.classes.AlignThread;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;
import org.firstinspires.ftc.teamcode.classes.TestThread.TestThread;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.concurrent.TimeUnit;

//-2.91
//-31.04
//3.65
@TeleOp(name = "BLUE AUTO")
@Config
public class RightParkCone extends LinearOpMode {

    public static double exposure = 10;
    public static int gain = 25;
    public static int WB = 6000;
    public void runOpMode() {
        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
        int AprilTagID =7;

        //Tag IDs
        int left = 6;
        int middle = 7;
        int right = 8;

        double distanceToPole = 0, distanceToCone = 0;

        boolean toggle1 = false;
        boolean toggle2 = false;

        double alignedTime = 0;
        double alignStartTime = 0;

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

        ElapsedTime timer = new ElapsedTime();

        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);

        DistanceSensor backDist, frontDist;
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
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPoseEstimate(new Pose2d(-31.425,64.75, Math.toRadians(90)));

//        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .addTemporalMarker(0, () -> {
//                    lift.setSlurpPower(1);
//                    pipeline.setPipelines("pole", "pole");
//                })
//                .splineToConstantHeading(new Vector2d(-34, 59.75), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-36, 57.75), Math.toRadians(-90))
//                .lineToSplineHeading(new Pose2d(-36, 52, Math.toRadians(-135)))
//                .addTemporalMarker(.5, () -> {
//                    lilArmL.setPosition(.382);
//                })
//                .lineToSplineHeading(new Pose2d(-36, 45, Math.toRadians(-45)),
//                        SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(2, () -> {
//                    lilArmL.setPosition(.047);
//                })
//                .lineToLinearHeading(new Pose2d(-36,24, Math.toRadians(-90)))
//                .lineToSplineHeading(new Pose2d(-36,12, Math.toRadians(-220)))
//                .build();

//        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .addTemporalMarker(0, () -> {
//                    lift.setSlurpPower(1);
//                    pipeline.setPipelines("pole", "pole");
//                })
//                .splineToConstantHeading(new Vector2d(-32, 64.75), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-12, 57.75), Math.toRadians(-90))
//                .lineTo(new Vector2d(-12, 16),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(15))
//                .splineTo(new Vector2d(-14,14), Math.toRadians(180))
//                .lineToLinearHeading(new Pose2d(-24,14, Math.toRadians(-180)))
//                .lineToSplineHeading(new Pose2d(-36,14, Math.toRadians(-220)))
//                .build();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.setSlurpPower(1);
                    pipeline.setPipelines("pole", "pole");
                })
                .lineToConstantHeading(new Vector2d(-34, 64.75))
                .splineToConstantHeading(new Vector2d(-36, 62.75), Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-34,3, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-34, 7.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36,24, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-36,12, Math.toRadians(135)))
                .build();

        alignerThread.start();

        while(!alignerThread.isAlive()){
            telemetry.addLine("Creating aligner thread");
            telemetry.update();
        }

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
                pipeline.setPipelines("sleeve", "pole");
                pipeline.backPoleDetector.setColors(true, false, false);
                pipeline.frontPoleDetector.setColors(false, false, true);
                aligner.camera.pointCam(.6, .6);
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

            AprilTagID = pipeline.AprilTagID(true);

            telemetry.addData("Sleeve position", AprilTagID);
            telemetry.addData("exposure", exposure);
            telemetry.addData("gain", gain);
            telemetry.addData("WB", WB);
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        pipeline.setPipelines("sleeve", "pole");
        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);
        aligner.camera.pointCam(.6, .6);

        waitForStart();
        timer.reset();

        drive.followTrajectorySequence(traj);

        lift.lift(1850, true);

        aligner.camera.enableCam(false);
        aligner.engageMaster(5, false, 135, true);

        startingPose = drive.getPoseEstimate();

        double AlignerTime = timer.time(TimeUnit.MILLISECONDS);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(alignedTime < 500 /** && (timer.time(TimeUnit.MILLISECONDS) - AlignerTime) < 3000*/) {
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

            if(aligner.camera.getAngle(false) < 10){
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

        for(int i = 0; i < 5; i++) {
            TrajectorySequence pickupcone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1000, false);
                        lift.setSlurpPower(1);
                        aligner.camera.pointCam(.7, .6);
                    })
                    .setTangent(Math.toRadians(135))
                    .splineToSplineHeading(new Pose2d(-48, 10, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            drive.followTrajectorySequence(pickupcone);

            aligner.camera.enableCam(true);
            aligner.engageMaster(2, true, 180, false);

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

            lift.lift(350 - i * 100, false);
            sleep(500);
            lift.lift(1000, false);

            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
                        lift.lift(500,false);
                        aligner.camera.pointCam(.5, .7);
                    })
                    .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(135)))
                    .build();

            drive.followTrajectory(deliver);

            lift.lift(1850, true);

            aligner.camera.enableCam(false);
            aligner.engageMaster(5, false, 135, true);

            startingPose = drive.getPoseEstimate();

            alignedTime = 0;

            AlignerTime = timer.time(TimeUnit.MILLISECONDS);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(alignedTime < 500 /** && (timer.time(TimeUnit.MILLISECONDS) - AlignerTime) < 3000*/) {
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

                if(aligner.camera.getAngle(false) < 10){
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

        /**
        if(AprilTagID == left) {
            Trajectory parkL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.drop();
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
                        lift.drop();
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .build();

            drive.followTrajectory(park);
        }else if(AprilTagID == right) {
            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.drop();
                    })
                    .lineToLinearHeading(new Pose2d(-34,12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-60,12, Math.toRadians(0)))
                    .build();

            drive.followTrajectorySequence(park);
        }
         */

        double time = timer.time(TimeUnit.SECONDS);

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

        while(opModeIsActive()){
            telemetry.addData("time", time);
            telemetry.addData("y movement", Math.abs(startingPose.getY() - currentPose.getY()));
            telemetry.addData("Xdist", aligner.xDist);
            telemetry.addData("Ydist", aligner.yDist);
            telemetry.update();
        }

//        liftThread.interrupt();
        aligner.stopCamera();
        alignerThread.interrupt();
    }
}
