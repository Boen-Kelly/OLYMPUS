package org.firstinspires.ftc.teamcode.Autos.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.oldcode.tests.classes.SignalSleeve;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous
public class LeftPark extends LinearOpMode {
    public void runOpMode() {
        DcMotor lift1, lift2;

        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        lift2 = hardwareMap.get(DcMotor.class, "Lift2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        SignalSleeve detector = new SignalSleeve(hardwareMap, "Webcam 1");
        SignalSleeve.DuckPos pos = SignalSleeve.DuckPos.THREE;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(36,64.75, Math.toRadians(-90)));

        while(!isStarted()) {
            pos = detector.getPosition();

            lift1.setTargetPosition(-400);
            lift2.setTargetPosition(lift1.getTargetPosition());
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(lift1.getMode());
            lift1.setPower(.75);
            lift2.setPower(lift1.getPower());

            telemetry.addData("Detecting", pos);
            telemetry.addLine("waiting for start");
            telemetry.addData("liftPos", lift1.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(36, 24))
                .lineTo(new Vector2d(36, 36))
                .turn(Math.toRadians(-90))
                .build();

        drive.followTrajectorySequence(traj);

        if(pos.equals(SignalSleeve.DuckPos.THREE)) {
            Trajectory trajL = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(13, 36))
                    .build();

            drive.followTrajectory(trajL);

            drive.turn(Math.toRadians(-90));
        }else if(pos.equals(SignalSleeve.DuckPos.TWO)) {
        }else if(pos.equals(SignalSleeve.DuckPos.ONE)) {
            Trajectory trajR = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(60, 36))
                    .build();

            drive.followTrajectory(trajR);
        }

        try {
            File test = new File("/sdcard/FIRST/nums.txt");
            FileWriter writer = new FileWriter("/sdcard/FIRST/nums.txt");
            writer.write(Math.toDegrees(drive.getPoseEstimate().getHeading()) + "");
            writer.close();
            telemetry.addLine("successfully wrote!");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("couldn't create file");
            telemetry.update();
            e.printStackTrace();
        }
    }
}
