package org.firstinspires.ftc.teamcode.Autos.Tests;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;
import org.firstinspires.ftc.teamcode.classes.LiftArm;
import org.firstinspires.ftc.teamcode.classes.MLToolChain;
import org.firstinspires.ftc.teamcode.classes.RoadRunnerThread;
import org.firstinspires.ftc.teamcode.classes.SignalSleeve;
import org.openftc.apriltag.AprilTagDetection;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous(name = "z.RRMidCalc")
public class RRMidCalc extends LinearOpMode{
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.425, 64.75, Math.toRadians(-90)));
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .build();

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        drive.followTrajectory(traj);

        waitForButton(drive);

        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-14, 0), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj2);

        waitForButton(drive);

        Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj3);

        waitForButton(drive);

        Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj4);

        waitForButton(drive);

        Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-10, -40), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj5);

        Trajectory traj6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(90)))
                .build();
        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-10, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj7);

        waitForButton(drive);

        telemetry.addData("EquationError X", traj7.end().getX() - drive.getPoseEstimate().getX());
        telemetry.addData("EquationError Y", traj7.end().getY() - drive.getPoseEstimate().getY());
        telemetry.addData("lastError", drive.getLastError());

        telemetry.update();

        sleep(10000);

    }

    public void waitForButton(SampleMecanumDrive drive){
        //while(!gamepad1.a && opModeIsActive()){
            //sleep(10);
            telemetry.addData("lastError", drive.getLastError());
            telemetry.update();
        //}
    }
}
