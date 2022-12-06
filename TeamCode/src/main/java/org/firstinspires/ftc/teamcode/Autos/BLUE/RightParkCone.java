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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

//-2.91
//-31.04
//3.65
@Autonomous
public class RightParkCone extends LinearOpMode {
    public void runOpMode() {
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");
        LiftArm lift = new LiftArm(hardwareMap);
        Thread liftThread = new Thread(lift);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36,64.75, Math.toRadians(-90)));

        while(!isStarted()) {
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();
        liftThread.start();

        lift.setSlurpPower(1);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, 7))
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(-135))
                .build();

        drive.followTrajectorySequence(traj);

        pipeline.align();

        Trajectory firstDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.lift(1500,false);
                })
                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(45)*11, drive.getPoseEstimate().getY() - Math.sin(45)*11, Math.toRadians(135)))
                .build();

        drive.followTrajectory(firstDeliver);

        lift.setSlurpPower(-1);
        sleep(500);

        for(int i = 0; i < 1; i++){
            TrajectorySequence pickup = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.drop();
                        lift.setSlurpPower(0);
                    })
                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() - Math.cos(45)*11, drive.getPoseEstimate().getY() + Math.sin(45)*11, drive.getPoseEstimate().getHeading()))
                    .addTemporalMarker(2, () -> {
                        lift.lift(1000,true);
                        lift.setSlurpPower(1);
                    })
                    .turn(Math.toRadians(45))
                    .lineToLinearHeading(new Pose2d(-57,12, Math.toRadians(180)))
                    .build();

            drive.followTrajectorySequence(pickup);

            lift.drop(500);
            sleep(500);
            lift.lift();

//            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
////                    .splineTo(new Vector2d(-48,12), Math.toRadians(180))
//                    .splineTo(new Vector2d(-36,12), Math.toRadians(-45))
//                    .build();

            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(.5, () -> {
                        lift.drop();
                    })
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .turn(Math.toRadians(-45))
                    .build();

            drive.followTrajectorySequence(deliver);

            pipeline.align();

            Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        lift.lift(1500, false);
                    })
                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(45)*11, drive.getPoseEstimate().getY() - Math.sin(45)*11, Math.toRadians(135)))
                    .build();

            drive.followTrajectory(backup);

            lift.setSlurpPower(-1);
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    lift.setSlurpPower(0);
                    lift.drop();
                })
                .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .build();

        drive.followTrajectorySequence(park);

//        if(pos.equals(SignalSleeve.DuckPos.ONE)) {
//            Trajectory trajL = drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .lineTo(new Vector2d(-10, 38))
//                    .build();
//
//            drive.followTrajectory(trajL);
//
//            drive.turn(Math.toRadians(90));
//        }else if(pos.equals(SignalSleeve.DuckPos.TWO)) {
//        }else if(pos.equals(SignalSleeve.DuckPos.THREE)) {
//            Trajectory trajR = drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .lineTo(new Vector2d(-60, 38))
//                    .build();
//
//            drive.followTrajectory(trajR);
//        }

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
