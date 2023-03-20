package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.DuckDetectorPipelineBlue;


@Autonomous(name = "Main Blue Auto", group = "Blue")
@Disabled
public class Main extends LinearOpMode {
    public void runOpMode(){
        DuckDetectorPipelineBlue detector = new DuckDetectorPipelineBlue(hardwareMap, "Webcam 1");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DuckDetectorPipelineBlue.DuckPos duckPos;

        Arm arm = new Arm(hardwareMap);

        DcMotorEx duckWheel = hardwareMap.get(DcMotorEx.class, "Rim");

        drive.setPoseEstimate(new Pose2d(-41.75,65.38, Math.toRadians(270)));

        while(!isStarted()) {
            telemetry.addLine("Waiting for start");
            telemetry.addData("webcam telemetry", detector);
            telemetry.addData("guess", detector.getPosition());
            telemetry.update();
        }

        duckPos = detector.getPosition();

        waitForStart();

        Trajectory wheel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62.38,57.25, Math.toRadians(180)))
                .build();

        drive.followTrajectory(wheel);

        Trajectory strafe = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62.38, 63, Math.toRadians(180)))
                .build();

        drive.followTrajectory(strafe);

        duckWheel.setVelocity(180, AngleUnit.DEGREES);
        sleep(3000);
        duckWheel.setVelocity(0, AngleUnit.DEGREES);

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.38, 12, Math.toRadians(180)))
                .build();

        drive.followTrajectory(park);

        Trajectory bottom = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(-12,-10, Math.toRadians(270)), 0)
                .addTemporalMarker(1, () -> {
                    arm.moveArm(-150, 1);
                })
                .build();

//        Trajectory followUp = drive.trajectoryBuilder(bottom.end(), true)
//                .lineToLinearHeading(new Pose2d(-12, -6, Math.toRadians(270)))
//                .build();

        Trajectory middle = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(-12,-9.38, Math.toRadians(270)), 0)
                .addTemporalMarker(.5, () -> {
                    arm.open(.25, 1);
                })
                .addTemporalMarker(1, () -> {
                    arm.moveArm(-250, 1);
                })
                .addTemporalMarker(2, () ->{
                    arm.open();
                })
                .build();

        Trajectory top = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineToLinearHeading(new Pose2d(-12,-3.38, Math.toRadians(270)), 0)
                .addTemporalMarker(.5, () -> {
                    arm.moveArm(-350, 1);
                })
                .build();


        if(duckPos.equals(DuckDetectorPipelineBlue.DuckPos.LEFT)) {
            drive.followTrajectory(bottom);
//            drive.followTrajectory(followUp);
            arm.open();
            sleep(500);
            arm.close();
            sleep(250);
        }else if(duckPos.equals(DuckDetectorPipelineBlue.DuckPos.CENTER)){
            drive.followTrajectory(middle);
            sleep(500);
            arm.open(.1,0);
            sleep(500);
            arm.close();
            sleep(250);
        }else{
            drive.followTrajectory(top);
            arm.open();
            sleep(500);
            arm.close();
            sleep(250);
        }

        Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12,-15, Math.toRadians(270)))
                .build();

        drive.followTrajectory(backup);

        Trajectory end = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.38, 31, Math.toRadians(0)))
                .addTemporalMarker(2, () -> {
                    arm.moveArm(0, .5);
                    arm.close();
                })
                .build();

        drive.followTrajectory(end);
    }
}
