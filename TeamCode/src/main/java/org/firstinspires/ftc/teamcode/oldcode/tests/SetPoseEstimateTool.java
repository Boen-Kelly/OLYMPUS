package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class SetPoseEstimateTool extends LinearOpMode{
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        //The idea behind this class, is to find the pose which is the starting
        //pose for your autonomous.
        //This class is subject to change.

        Trajectory poseEstimate = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0,0,0))
                .build();

        drive.followTrajectory(poseEstimate);
    }
}
