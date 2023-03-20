package org.firstinspires.ftc.teamcode.Autos.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.SameLen;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;
import org.firstinspires.ftc.teamcode.classes.MLToolChain;
import org.firstinspires.ftc.teamcode.classes.RoadRunnerThread;
import org.firstinspires.ftc.teamcode.classes.SignalSleeve;
import org.openftc.apriltag.AprilTagDetection;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous(name = "ThreadTestingRR")
public class RRThreadTest extends LinearOpMode{

    //public static Pose2d poseWithError1 = new Pose2d(0,0,0);



    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-31.425,64.75, Math.toRadians(-90)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-32, 64.75))
                .splineToConstantHeading(new Vector2d(-34, 62.75), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-34,3, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-34, 7.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-34,12, Math.toRadians(140)))
                .build();




        RoadRunnerThread RRThread = new RoadRunnerThread(hardwareMap, drive, traj,1);
        Thread RoadThread = new Thread(RRThread);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        RoadThread.start();
        RRThread.currentTraj = traj;
        RRThread.trajNum = 1;
        drive.followTrajectorySequence(traj);

        RRThread.currentTraj = RRThread.calculatedTraj;
        RRThread.trajNum = 2;
        drive.followTrajectorySequence(RRThread.calculatedTraj);

        RRThread.currentTraj = RRThread.calculatedTraj;
        RRThread.trajNum = 3;
        drive.followTrajectorySequence(RRThread.calculatedTraj);

        RRThread.currentTraj = RRThread.calculatedTraj;
        drive.followTrajectorySequence(RRThread.calculatedTraj);

        RoadThread.interrupt();
    }
}
