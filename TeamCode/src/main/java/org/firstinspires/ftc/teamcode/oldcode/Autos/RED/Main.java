package org.firstinspires.ftc.teamcode.Autos.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.Arm;


@Autonomous(name = "Main Red Auto", group = "Red")
@Disabled
public class Main extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx duckWheel = hardwareMap.get(DcMotorEx.class, "Rim");

        Arm arm = new Arm(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-41.38,-65.75, Math.toRadians(180)));

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        Trajectory wheel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.38,-57.25, Math.toRadians(270)))
                .build();

        drive.followTrajectory(wheel);

        Trajectory strafe = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(3)
                .build();

        drive.followTrajectory(strafe);

        duckWheel.setVelocity(-180, AngleUnit.DEGREES);
        sleep(3000);
        duckWheel.setVelocity(0, AngleUnit.DEGREES);

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.38, -12, Math.toRadians(180)))
                .build();

        drive.followTrajectory(park);

        Trajectory deliver = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(-12,-3.38, Math.toRadians(90)), 0)
                .addTemporalMarker(1, () -> {
                    arm.moveArm(-350, .75);
                })
                .build();

        drive.followTrajectory(deliver);
        arm.open();

        Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(4)
                .build();

        drive.followTrajectory(backup);

        Trajectory end = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.38, -36, Math.toRadians(0)))
                .addTemporalMarker(2, () -> {
                    arm.moveArm(0, .5);
                    arm.close();
                })
                .build();

        drive.followTrajectory(end);
    }
}
