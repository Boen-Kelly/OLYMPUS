package org.firstinspires.ftc.teamcode.Autos.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.Arm;

@Disabled
@Autonomous (name = "get blocks red", group = "Red")
public class GetBlocksRed extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        drive.setPoseEstimate(new Pose2d(6.25, -65.38, Math.toRadians(90)));

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();

        Trajectory lineUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-1.89, -39.34, Math.toRadians(-52.32)))
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-350,1);
                })
                .build();

        drive.followTrajectory(lineUp);

        arm.open();
        sleep(250);
        arm.close();

        Trajectory collect1 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                .splineTo(new Vector2d(24, -65), 0)
                .splineToConstantHeading(new Vector2d(52,-65),0)
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(0, .5);
                    intake.setPower(-1);
                    arm.open();
                })
                .build();

        drive.followTrajectory(collect1);

        arm.close();
        sleep(250);
        intake.setPower(1);

        Trajectory backup1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(24,-65, 0))
                .build();

        drive.followTrajectory(backup1);

        Trajectory deliver1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-1.89, -39.34), Math.toRadians(-232.32))
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-350, 1);
                    intake.setPower(0);
                })
                .build();

        drive.followTrajectory(deliver1);

        arm.open();
        sleep(1000);
        arm.close();

        Trajectory collect2 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                .splineTo(new Vector2d(24, -65), 0)
                .splineToConstantHeading(new Vector2d(55,-65), 0)
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(0, .5);
                    intake.setPower(-1);
                    arm.open();
                })
                .build();

        drive.followTrajectory(collect2);

        arm.close();
        sleep(250);
        intake.setPower(1);

        Trajectory backup2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(24,-65), Math.toRadians(180))
                .build();

        drive.followTrajectory(backup2);

        Trajectory deliver2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-1.89, -39.34), Math.toRadians(-232.32))
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-350, 1);
                    intake.setPower(0);
                })
                .build();

        drive.followTrajectory(deliver2);

        arm.open();
        sleep(1000);
        arm.close();

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(24, -65), 0)
                .lineToLinearHeading(new Pose2d(43.38, -65, 0))
                .addTemporalMarker(2, () -> {
                    arm.moveArm(0, .5);
                })
                .build();

        drive.followTrajectory(park);
    }
}
