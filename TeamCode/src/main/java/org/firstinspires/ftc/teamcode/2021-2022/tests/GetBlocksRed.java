package org.firstinspires.ftc.teamcode.tests;

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
import org.firstinspires.ftc.teamcode.classes.DuckDetectorPipelineRed;

@Disabled
@Autonomous
public class GetBlocksRed extends LinearOpMode {
    public void runOpMode(){

//        double armHeight = -350;

//        DuckDetectorPipelineRed.DuckPos position = DuckDetectorPipelineRed.DuckPos.LEFT;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
//        DuckDetectorPipelineRed detector = new DuckDetectorPipelineRed(hardwareMap, "Webcam 1");

        drive.setPoseEstimate(new Pose2d(6.25, -65.38, Math.toRadians(90)));

//        while (!isStarted()) {
//            telemetry.addLine("waiting for start");
//            telemetry.addData("webcam data", detector);
//            telemetry.update();
//            position = detector.getPosition();
//        }

//        waitForStart();

//        if(position == DuckDetectorPipelineRed.DuckPos.LEFT){
//            armHeight = -350;
//        }else if(position == DuckDetectorPipelineRed.DuckPos.CENTER){
//            armHeight = -240;
//        }else if(position == DuckDetectorPipelineRed.DuckPos.RIGHT){
//            armHeight = -350;
//        }
//
//        double copiedArmHeight = armHeight;

        Trajectory lineUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-1.89, -39.34, Math.toRadians(-52.32)))
                .addTemporalMarker(1, () -> {
                    arm.moveArm(-350,1);
                })
                .build();

        Trajectory collect1 = drive.trajectoryBuilder(lineUp.end(), false)
                .splineTo(new Vector2d(24, -64), 0)
                .splineTo(new Vector2d(47, -64), Math.toRadians(0))
                .addTemporalMarker(.5, () -> {
                    intake.setPower(-1);
                    arm.open();
                })
                .addTemporalMarker(1, () -> {
                    arm.moveArm(0, .5);
                })
                .build();

        Trajectory deliver1 = drive.trajectoryBuilder(collect1.end(), true)
                .splineTo(new Vector2d(24,-64), Math.toRadians(180))
                .splineTo(new Vector2d(-1.89, -39.34), Math.toRadians(-232.32))
                .addTemporalMarker(0, () -> {
                    arm.close();
                })
                .addTemporalMarker(.5, () -> {
                    arm.moveArm(-350, 1);
                    intake.setPower(1);
                })
                .addTemporalMarker(2.5, () ->{
                    intake.setPower(0);
                })
                .build();

        Trajectory collect2 = drive.trajectoryBuilder(deliver1.end(), false)
                .splineTo(new Vector2d(24, -64), 0)
                //.lineToLinearHeading(new Pose2d(49 + addedXDistantce, -65 - addedYDistance, Math.toRadians(15)))
                .splineToConstantHeading(new Vector2d(49, -60), Math.toRadians(0))
                .addTemporalMarker(.5, () -> {
                    intake.setPower(-1);
                    arm.open();
                })
                .addTemporalMarker(1, () -> {
                    arm.moveArm(0, .5);
                })
                .build();

        Trajectory deliver2 = drive.trajectoryBuilder(collect2.end(), true)
                .splineTo(new Vector2d(24,-64), Math.toRadians(180))
                .splineTo(new Vector2d(-1.89, -39.34), Math.toRadians(-232.32))
                .addTemporalMarker(0, () -> {
                    arm.close();
                })
                .addTemporalMarker(.5, () -> {
                    arm.close();
                    arm.moveArm(-350, 1);
                    intake.setPower(1);
                })
                .addTemporalMarker(2.5, () ->{
                    intake.setPower(0);
                })
                .build();

        Trajectory collect3 = drive.trajectoryBuilder(deliver2.end(), false)
                .splineTo(new Vector2d(24, -64), 0)
                //.lineToLinearHeading(new Pose2d(49 + addedXDistantce, -65 - addedYDistance, Math.toRadians(15)))
                .splineToConstantHeading(new Vector2d(50, -58), Math.toRadians(0))
                .addTemporalMarker(.5, () -> {
                    intake.setPower(-1);
                    arm.open();
                })
                .addTemporalMarker(1, () -> {
                    arm.moveArm(0, .5);
                })
                .build();

        Trajectory deliver3 = drive.trajectoryBuilder(collect3.end(), true)
                .splineTo(new Vector2d(24,-64), Math.toRadians(180))
                .splineTo(new Vector2d(-1.89, -39.34), Math.toRadians(-232.32))
                .addTemporalMarker(0, () -> {
                    arm.close();
                })
                .addTemporalMarker(.5, () -> {
                    arm.close();
                    arm.moveArm(-350, 1);
                    intake.setPower(1);
                })
                .addTemporalMarker(2.5, () ->{
                    intake.setPower(0);
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(deliver3.end())
                .splineTo(new Vector2d(24, -65), 0)
                .lineToLinearHeading(new Pose2d(43.38, -65, 0))
                .addTemporalMarker(1, () -> {
                    arm.moveArm(0, .5);
                    intake.setPower(-1);
                })
                .build();

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();

        drive.followTrajectory(lineUp);

        arm.open();
        sleep(250);
        arm.close();

        //Begin cycle one
        drive.followTrajectory(collect1);

        drive.followTrajectory(deliver1);

        arm.open();
        sleep(500);
        arm.close();

        //Begin cycle two
        drive.followTrajectory(collect2);

        drive.followTrajectory(deliver2);

        arm.open();
        sleep(500);
        arm.close();

        //Begin cycle three
        drive.followTrajectory(collect3);

        drive.followTrajectory(deliver3);

        arm.open();
        sleep(500);
        arm.close();

        drive.followTrajectory(park);
    }
}
