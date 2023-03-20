package org.firstinspires.ftc.teamcode.Autos.RED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;

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
@Autonomous(name = "RED AUTO")
@Config
public class RightParkCone extends LinearOpMode {
    public void runOpMode() {
        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
        int AprilTagID =7;

        //Tag IDs
        int left = 6;
        int middle = 7;
        int right = 8;

        double exposure = 25;
        int gain = 1;
        int WB = 5000;

        double distanceToPole = 0, distanceToCone = 0;

        boolean toggle1 = false;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime();

        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, true, false);

        DistanceSensor backDist, frontDist;
        Servo lilArmL, lilArmR;

        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        lilArmL = hardwareMap.get(Servo.class, "lilArm1");
        lilArmR = hardwareMap.get(Servo.class, "lilArm2");


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

        drive.setPoseEstimate(new Pose2d(-31.425,64.75, Math.toRadians(-90)));

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

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.setSlurpPower(1);
                    pipeline.setPipelines("pole", "pole");
                })
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, 57.75), Math.toRadians(-90))
                .lineTo(new Vector2d(-12, 16),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineTo(new Vector2d(-14,14), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-24,14, Math.toRadians(-180)))
                .lineToSplineHeading(new Pose2d(-36,14, Math.toRadians(-220)))
                .build();

        while(!isStarted() && !isStopRequested()) {
            if(gamepad1.a){
                pipeline.setPipelines("pole", "pole");
                pipeline.frontPoleDetector.setColors(true, false, false);
                pipeline.backPoleDetector.setColors(true, false, false);
//                pipeline.pointCam(true, .4);
            }else if(gamepad1.b){
                pipeline.setPipelines("sleeve", "sleeve");
//                pipeline.pointCam(true, .6);
            }else if(gamepad1.y){
                pipeline.setPipelines("sleeve", "pole");
                pipeline.backPoleDetector.setColors(true, false, false);
                pipeline.frontPoleDetector.setColors(false, true, false);
//                pipeline.pointCam(true, .6);
            }

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
        pipeline.frontPoleDetector.setColors(false, true, false);
//        pipeline.pointCam(true, .6);

        waitForStart();
        timer.reset();

        drive.followTrajectory(traj);

        lift.lift(1850, false);

//        pipeline.turnToAlign(.79, false);

        distanceToPole = backDist.getDistance(DistanceUnit.INCH);

        if(distanceToPole > 40){
            distanceToPole = 14;
        }

//        telemetry.addData("Distance to pole", distanceToPole);
//        telemetry.update();
//        sleep(1000);
        drive.update();

        Trajectory firstDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getHeading()))
                .back(distanceToPole - 2,
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        drive.followTrajectory(firstDeliver);

        lift.setSlurpPower(-1);
//        sleep(500);

        for(int i = 0; i < 1; i++){
//            TrajectorySequence pickup = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(0, () -> {
//                        lift.lift(0, true);
//                        lift.setSlurpPower(0);
//                    })
//                    .forward(2-distanceToPole)
//                    .addTemporalMarker(1, () -> {
//                        pipeline.strafeToAlign(.8, true);
//                    })
//                    .addTemporalMarker(2.5, () -> {
//                        lift.lift(1000,true);
//                        lift.setSlurpPower(1);
//                    })
//                    .build();

            Trajectory pickupcone = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
                        lift.lift(0, false);
                        lift.setSlurpPower(0);
                    })
                    .forward(distanceToPole - 4)
                    .splineTo(new Vector2d(-40, 14), Math.toRadians(180))
                    .build();

            drive.followTrajectory(pickupcone);

//            pipeline.turnToAlign(.525, true);

//            sleep(500);
            distanceToCone = frontDist.getDistance(DistanceUnit.INCH);

            if(distanceToCone > 40){
                distanceToCone = 26;
            }

//            telemetry.addData("dist", distanceToCone);
//            telemetry.update();
            drive.update();

            Trajectory collect = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1000, true);
                        lift.setSlurpPower(1);
                    })
                    .forward(distanceToCone-2)
                    .build();

            drive.followTrajectory(collect);

//            lift.drop(350);
            sleep(500);
//            lift.lift();

            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
//                        lift.drop(500);
                    })
                    .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(135)))
                    .build();
//            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(.5, () -> {
//                        lift.drop();
//                    })
//                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
//                    .turn(Math.toRadians(-45))
//                    .build();

            drive.followTrajectory(deliver);


            telemetry.addLine("aligning");
            telemetry.update();

//            pipeline.turnToAlign(.77, false);
//            sleep(500);
            distanceToPole = backDist.getDistance(DistanceUnit.INCH);

            if(distanceToPole > 40){
                distanceToPole = 14;
            }

            drive.update();

            Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1850, false);
                    })
                    .back(distanceToPole-2,
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(20))
                    .build();

            drive.followTrajectory(backup);

            lift.setSlurpPower(-1);
        }

        if(AprilTagID == left) {
            Trajectory parkL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
//                        lift.drop();
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
//                        lift.drop();
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .build();

            drive.followTrajectory(park);
        }else if(AprilTagID == right) {
            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
//                        lift.drop();
                    })
                    .lineToLinearHeading(new Pose2d(-34,12, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-60,12, Math.toRadians(0)))
                    .build();

            drive.followTrajectorySequence(park);
        }

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
            telemetry.update();
        }

    }
}
