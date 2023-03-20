package org.firstinspires.ftc.teamcode.Autos.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Red park short", group = "Red")
@Disabled
public class ParkShort extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,-65.75,0));

        waitForStart();

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36,-65, Math.toRadians(0)))
                .build();

        drive.followTrajectory(park);

        Trajectory strafe = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40,-36))
                .build();

        drive.followTrajectory(strafe);

        Trajectory end = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(60,-40), Math.toRadians(270))
                .build();

        drive.followTrajectory(end);


    }
}
