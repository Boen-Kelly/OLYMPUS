package org.firstinspires.ftc.teamcode.Autos.BLUE;

import android.text.Html;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.DuckDetectorPipelineBlue;

@Autonomous(name = "Get blocks blue", group = "Blue")
@Disabled
public class GetBlocksBlue extends LinearOpMode {
    public void runOpMode(){
        DuckDetectorPipelineBlue.DuckPos duckPos;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        DuckDetectorPipelineBlue detector = new DuckDetectorPipelineBlue(hardwareMap, "Webcam 1");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        drive.setPoseEstimate(new Pose2d(6.25, 64.88, Math.toRadians(270)));

        Trajectory bottom = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(1.75, 44.93, Math.toRadians(56.35)))
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-175,1);
                })
                .build();

        Trajectory middle = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(1.62, 45.66, Math.toRadians(56.35)))
                .addTemporalMarker(.5, () ->{
                    arm.open(.25, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-250,1);
                })
                .addTemporalMarker(2, () ->{
                    arm.open();
                })
                .build();

        Trajectory top = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-1.89, 39.34, Math.toRadians(52.32)))
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(-350,1);
                })
                .build();

        while(!isStarted()) {
            String str = "<h1 style=\"text-align: center;\"><strong><span style=\"color: #ff0000;\">BLUE</span></strong></h1>";
            telemetry.addData("side", Html.fromHtml(str));
            telemetry.addData("webcam telemetry", detector);
            telemetry.addData("guess", detector.getPosition());
            telemetry.update();
        }

        duckPos = detector.getPosition();

        waitForStart();

        if(duckPos.equals(DuckDetectorPipelineBlue.DuckPos.LEFT)) {
            drive.followTrajectory(bottom);
            arm.open();
            sleep(250);
            arm.close();
            sleep(250);
        }else if(duckPos.equals(DuckDetectorPipelineBlue.DuckPos.CENTER)){
            drive.followTrajectory(middle);
            sleep(250);
            arm.open(.1,0);
            sleep(250);
            arm.close();
            sleep(250);
        }else{
            drive.followTrajectory(top);
            arm.open();
            sleep(250);
            arm.close();
            sleep(250);
        }

        Trajectory collect1 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                .splineTo(new Vector2d(24, 65), 0)
                .lineToLinearHeading(new Pose2d(49, 65, 0))
                .addTemporalMarker(1.5, () -> {
                    intake.setPower(-1);
                    arm.moveArm(0, 1);
                    arm.open(.37,1);
                })
                .build();

        drive.followTrajectory(collect1);

        Trajectory backup1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(24,65, 0))
                .addTemporalMarker(0, () ->{
                    arm.close();
                })
                .addTemporalMarker(.25, () ->{
                    intake.setPower(1);
                })
                .build();

        drive.followTrajectory(backup1);

        Trajectory deliver1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-.89, 41.34), Math.toRadians(232.32))
                .addTemporalMarker(.5, () ->{
                    arm.moveArm(-350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    intake.setPower(0);
                })
                .build();

        drive.followTrajectory(deliver1);

        arm.open();
        sleep(250);
        arm.close();

        Trajectory collect2 = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                .splineTo(new Vector2d(24, 65), 0)
                .splineToConstantHeading(new Vector2d(52,65), 0)
                .addTemporalMarker(1.5, () -> {
                    arm.moveArm(0, .5);
                    intake.setPower(-1);
                    arm.open(.37,1);
                })
                .build();

        drive.followTrajectory(collect2);

        Trajectory backup2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(24,65),Math.toRadians(180))
                .addTemporalMarker(0, () ->{
                    arm.close();
                })
                .addTemporalMarker(.25, () -> {
                    intake.setPower(1);
                })
                .build();

        drive.followTrajectory(backup2);

        Trajectory deliver2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(-.89, 41.34), Math.toRadians(232.32))
                .addTemporalMarker(.5, () ->{
                    arm.moveArm(-350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    intake.setPower(0);
                })
                .build();

        drive.followTrajectory(deliver2);

        arm.open();
        sleep(250);
        arm.close();

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(24, 65), 0)
                .lineToLinearHeading(new Pose2d(43.38, 65, 0))
                .addTemporalMarker(2, () -> {
                    arm.moveArm(0, .5);
                    intake.setPower(-1);
                    arm.open();
                })
                .build();

        drive.followTrajectory(park);
    }
}
