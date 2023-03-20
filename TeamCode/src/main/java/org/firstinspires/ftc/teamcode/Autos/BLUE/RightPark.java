package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;
import java.util.concurrent.locks.ReentrantReadWriteLock;

@Autonomous
@Disabled
public class RightPark extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36,64.75, Math.toRadians(-90)));

        String parkVar = "LEFT";

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();
//        Trajectory forward = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .splineTo(new Vector2d(-48, 36), Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-36,36, Math.toRadians(180)))
//                .build();
//
//        drive.followTrajectory(forward);
//
//
//        Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(-255)))
//                .build();
//
//        drive.followTrajectory(deliver);
//
//
//        drive.turn(Math.toRadians(90));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, 24))
                .lineTo(new Vector2d(-36, 36))
                .turn(Math.toRadians(90))
                .build();

        drive.followTrajectorySequence(traj);

        if(parkVar.equals("LEFT")) {
            Trajectory trajL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-10, 38))
                    .build();

            drive.followTrajectory(trajL);

            drive.turn(Math.toRadians(90));
        }else if(parkVar.equals("MIDDLE")) {
        }else if(parkVar.equals("RIGHT")) {
            Trajectory trajR = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-60, 38))
                    .build();

            drive.followTrajectory(trajR);
        }
    }
}
