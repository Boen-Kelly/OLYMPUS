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
//        DcMotor lift1, lift2;
//
//        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
//        lift2 = hardwareMap.get(DcMotor.class, "Lift2");
//
//        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36,64.75, Math.toRadians(-90)));

        while(!isStarted()) {
            telemetry.addLine("waiting for start");
            telemetry.update();
        }

        waitForStart();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, 7))
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(-135))
                .build();

        drive.followTrajectorySequence(traj);

        pipeline.align();

        Trajectory firstDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(45)*5, drive.getPoseEstimate().getY() - Math.sin(45)*5, Math.toRadians(135)))
                .build();

        drive.followTrajectory(firstDeliver);

        for(int i = 0; i < 1; i++){
            TrajectorySequence pickup = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .lineTo(new Vector2d(-60,12))
                    .build();

            drive.followTrajectorySequence(pickup);

//            Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
////                    .splineTo(new Vector2d(-48,12), Math.toRadians(180))
//                    .splineTo(new Vector2d(-36,12), Math.toRadians(-45))
//                    .build();

            TrajectorySequence deliver = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-36,12, Math.toRadians(180)))
                    .turn(Math.toRadians(-45))
                    .build();

            drive.followTrajectorySequence(deliver);

            pipeline.align();

            Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(45)*5, drive.getPoseEstimate().getY() - Math.sin(45)*5, Math.toRadians(135)))
                    .build();

            drive.followTrajectory(backup);
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
    }
}
