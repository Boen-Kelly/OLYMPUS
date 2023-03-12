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
        TrajectoryBuilder traj2 = drive.trajectoryBuilder(RoadRunnerThread.poseWithError1)
                .splineToConstantHeading(new Vector2d(-14, 0), Math.toRadians(-90));
        TrajectoryBuilder traj3 = drive.trajectoryBuilder(RoadRunnerThread.poseWithError1)
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(-90));

        drive.setPoseEstimate(new Pose2d(-31.425,64.75, Math.toRadians(-90)));

        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .build();




        RoadRunnerThread RRThread = new RoadRunnerThread(hardwareMap, drive, traj, traj2,1);
        Thread RoadThread = new Thread(RRThread);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        RoadThread.start();
        drive.followTrajectory(traj);

        RoadRunnerThread RRThread2 = new RoadRunnerThread(hardwareMap,drive, RRThread.calculatedTraj, traj3, 2);
        Thread RoadThread2 = new Thread(RRThread2);

        RoadThread2.start();
        drive.followTrajectory(RRThread.calculatedTraj);

        RoadThread.interrupt();

        drive.followTrajectory(RRThread2.calculatedTraj);

        RoadThread2.interrupt();

    }
    public TrajectoryBuilder traj1Builder1(SampleMecanumDrive drive, Pose2d poseError){
        return drive.trajectoryBuilder(poseError)
                .splineToConstantHeading(new Vector2d(-14, 0), Math.toRadians(-90));
    }
    public TrajectoryBuilder traj1Builder2(SampleMecanumDrive drive, Pose2d poseError){
        return drive.trajectoryBuilder(poseError)
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(-90));
    }
}
