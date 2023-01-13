package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;
import org.firstinspires.ftc.teamcode.classes.MLToolChain;
import org.firstinspires.ftc.teamcode.classes.SignalSleeve;
import org.openftc.apriltag.AprilTagDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

//-2.91
//-31.04
//3.65
@Autonomous
public class RightParkCone extends LinearOpMode {
    public void runOpMode() {
        AutoAlignPipeline.DuckPos sleevePos = AutoAlignPipeline.DuckPos.ONE;
        int AprilTagID =7;

        //Tag IDs
        int left = 6;
        int middle = 7;
        int right = 8;
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        double heading1, heading2;

        while(!pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        LiftArm lift = new LiftArm(hardwareMap);
        Thread liftThread = new Thread(lift);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36,64.75, Math.toRadians(-90)));

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.setSlurpPower(1);
                })
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, 57.75), Math.toRadians(-90))
                .lineTo(new Vector2d(-12, 14))
                .splineTo(new Vector2d(-14,12), Math.toRadians(180))
                .lineTo(new Vector2d(-36,12))
                .build();

        while(!isStarted() && !isStopRequested()) {
            if(gamepad1.a){
                pipeline.useFrontCam();
            }else if(gamepad1.b){
                pipeline.useBackCam();
            }

            AprilTagID = pipeline.AprilTagID();

            telemetry.addData("Sleeve position", AprilTagID);
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();
        pipeline.useBackCam();
        liftThread.start();

        drive.followTrajectory(traj);
//        heading1 = Math.toDegrees(drive.getPoseEstimate().getHeading());

        drive.turn(Math.toRadians(-45));

        pipeline.turnToAlign(.75, false);

        drive.update();

        Trajectory firstDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.lift(1500,false);
                })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getHeading()))
                .back(11)
                .build();

        drive.followTrajectory(firstDeliver);

        lift.setSlurpPower(-1);
        sleep(500);

//        for(int i = 0; i < 1; i++){
//            TrajectorySequence pickup = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(0, () -> {
//                        lift.lift(0, false);
//                        lift.setSlurpPower(0);
//                    })
//                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() - Math.cos(45)*11, drive.getPoseEstimate().getY() + Math.sin(45)*11, drive.getPoseEstimate().getHeading()))
//                    .addTemporalMarker(2, () -> {
//                        lift.lift(1000,true);
//                        lift.setSlurpPower(1);
//                    })
//                    .turn(Math.toRadians(45))
//                    .lineToLinearHeading(new Pose2d(-59,12, Math.toRadians(180)))
//                    .build();
//
//            drive.followTrajectorySequence(pickup);
//
//            lift.drop(400);
//            sleep(1000);
//            lift.lift();
//
////            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
//////                    .splineTo(new Vector2d(-48,12), Math.toRadians(180))
////                    .splineTo(new Vector2d(-36,12), Math.toRadians(-45))
////                    .build();
//
//            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(.5, () -> {
//                        lift.drop();
//                    })
//                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
//                    .turn(Math.toRadians(-45))
//                    .build();
//
//            drive.followTrajectorySequence(deliver);
//
//            pipeline.align();
//
//            Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .addTemporalMarker(0, () -> {
//                        lift.lift(1500, false);
//                    })
//                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(45)*11, drive.getPoseEstimate().getY() - Math.sin(45)*11, drive.getPoseEstimate().getHeading()))
//                    .build();
//
//            drive.followTrajectory(backup);
//
//            lift.setSlurpPower(-1);
//        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(1, () -> {
                    lift.setSlurpPower(0);
                    lift.lift(0, false);
                })
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()))*10, drive.getPoseEstimate().getHeading()))
                .forward(11)
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(-36, 32, Math.toRadians(90)))
                .addTemporalMarker(2, () -> {
                    lift.drop();
                })
                .turn(Math.toRadians(-90))
                .build();

        drive.followTrajectorySequence(park);

        if(AprilTagID == left) {
            Trajectory trajL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-14, 32))
                    .build();

            drive.followTrajectory(trajL);

            drive.turn(Math.toRadians(90));
        }else if(AprilTagID == middle) {
        }else if(AprilTagID == right) {
            Trajectory trajR = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-60, 32))
                    .build();

            drive.followTrajectory(trajR);
        }

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

        liftThread.interrupt();
    }
}
