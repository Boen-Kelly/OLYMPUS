package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.tests.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.oldcode.tests.classes.LiftArm;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

//-2.91
//-31.04
//3.65
@Autonomous(name = "BLUE AUTO")
public class RightParkCone extends LinearOpMode {
    public void runOpMode() {
        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
        int AprilTagID =7;

        //Tag IDs
        int left = 6;
        int middle = 7;
        int right = 8;

        double distanceToPole = 0, distanceToCone = 0;

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime();

        pipeline.backPoleDetector.setColors(true, false, false);
        pipeline.frontPoleDetector.setColors(false, false, true);

        DistanceSensor backDist, frontDist;

        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");

        LiftArm lift = new LiftArm(hardwareMap);
        Thread liftThread = new Thread(lift);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-31.425,64.75, Math.toRadians(-90)));

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.setSlurpPower(1);
                    pipeline.setPipelines("pole", "pole");
                })
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, 57.75), Math.toRadians(-90))
                .lineTo(new Vector2d(-12, 14),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-14,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-24,12, Math.toRadians(-180)))
                .lineToSplineHeading(new Pose2d(-36,12, Math.toRadians(-220)))
                .build();


        while(!isStarted() && !isStopRequested()) {
            if(gamepad1.a){
                pipeline.setPipelines("pole", "pole");
            }else if(gamepad1.b){
                pipeline.setPipelines("sleeve", "sleeve");
            }else if(gamepad1.y){
                pipeline.setPipelines("sleeve", "pole");
            }

            AprilTagID = pipeline.AprilTagID(true);

            telemetry.addData("Sleeve position", AprilTagID);
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();
        timer.reset();
        liftThread.start();

        drive.followTrajectory(traj);
//        heading1 = Math.toDegrees(drive.getPoseEstimate().getHeading());

//        drive.turn(Math.toRadians(-40));

//        pipeline.turnToAlign(.77, false);

        distanceToPole = backDist.getDistance(DistanceUnit.INCH);

        if(distanceToPole > 40){
            distanceToPole = 14;
        }

//        telemetry.addData("Distance to pole", distanceToPole);
//        telemetry.update();
//        sleep(1000);
        drive.update();

        Trajectory firstDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.lift(1400,false);
                })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getHeading()))
                .back(distanceToPole - 3)
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
//                    .forward(distanceToPole - 4)
                    .splineTo(new Vector2d(-40, 12), Math.toRadians(180))
                    .build();

            drive.followTrajectory(pickupcone);

//            pipeline.turnToAlign(.83, true);

//            sleep(500);
            distanceToCone = frontDist.getDistance(DistanceUnit.INCH);

            if(distanceToCone > 40){
                distanceToCone = 26;
            }

            telemetry.addData("dist", distanceToCone);
            telemetry.update();
            drive.update();

            Trajectory collect = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1000, true);
                        lift.setSlurpPower(1);
                    })
                    .forward(distanceToCone-7)
                    .build();

            drive.followTrajectory(collect);

            lift.drop(350);
            sleep(1000);
            lift.lift();

            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
                        lift.drop(500);
                    })
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(135)))
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
                        lift.lift(1500, false);
                    })
                    .back(distanceToPole-3)
                    .build();

            drive.followTrajectory(backup);

            lift.setSlurpPower(-1);
        }

        if(AprilTagID == left) {
            Trajectory parkL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.drop();
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .splineTo(new Vector2d(-36, 12), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-24, 12), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-12, 10), Math.toRadians(0))
                    .splineTo(new Vector2d(-10,12),Math.toRadians(90))
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
            Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(1, () -> {
                        lift.setSlurpPower(0);
                        lift.drop();
                    })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                    .splineTo(new Vector2d(-34,10), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-48, 12), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-60,12), Math.toRadians(0))
                    .build();

            drive.followTrajectory(park);
        }

        double time = timer.time(TimeUnit.SECONDS);

        try {
            File test = new File("/sdcard/FIRST/nums.txt");
            FileWriter writer = new FileWriter("/sdcard/FIRST/nums.txt");
            writer.write(Math.toDegrees(drive.getPoseEstimate().getHeading()) + "");
            writer.close();
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

        liftThread.interrupt();
    }
}
