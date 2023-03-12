package org.firstinspires.ftc.teamcode.Autos.Tests;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "z.RRFulCalc")
public class RRFulSpline extends LinearOpMode{
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-31.425, 64.75, Math.toRadians(-90)));
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-14, 59.75), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-14, 0), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-10, -40), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, -32,Math.toRadians(90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-10, 0,Math.toRadians(0)), Math.toRadians(90))
                .build();


        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        drive.followTrajectory(traj);

        waitForButton(drive);


        telemetry.addData("EquationError X", traj.end().getX() - drive.getPoseEstimate().getX());
        telemetry.addData("EquationError Y", traj.end().getY() - drive.getPoseEstimate().getY());
        telemetry.addData("lastError", drive.getLastError());

        telemetry.update();
        sleep(10000);


    }

    public void waitForButton(SampleMecanumDrive drive){
        while(!gamepad1.a && opModeIsActive()){
            sleep(10);
            telemetry.addData("lastError", drive.getLastError());

            telemetry.update();
        }
    }
}
