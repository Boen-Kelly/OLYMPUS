package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autos.BLUE.RightParkCone;
import org.firstinspires.ftc.teamcode.Autos.Tests.RRThreadTest;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.lang.reflect.Method;

public class RoadRunnerThread implements Runnable{

    private SampleMecanumDrive drive;
    private TrajectorySequence traj;
    public TrajectorySequence calculatedTraj;
    public int trajNum;
    private HardwareMap hardwareMap;
    RRThreadTest auto = new RRThreadTest();
    public Pose2d poseWithError1 = new Pose2d(0,0,0);

    public TrajectorySequence currentTraj;

    public RoadRunnerThread(HardwareMap hardwareMap, SampleMecanumDrive drive, TrajectorySequence traj, int trajNum){
        this.drive = drive;
        this.traj = traj;
        this.trajNum = trajNum;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void run(){
        if(trajNum != 0){
            calculateNextTraj();
        }
    }

    private void calculateNextTraj(){
        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        lightsThread lights = new lightsThread(hardwareMap);

        timer.reset();


        while(timer.seconds() < currentTraj.duration()-0.2){
            dashboardTelemetry.addData("timer", timer.seconds());
        }
        poseWithError1 = new Pose2d(currentTraj.end().getX() + drive.getLastError().getX(),currentTraj.end().getY() + drive.getLastError().getY(),currentTraj.end().getHeading());



        if(trajNum == 1){
            calculatedTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX() + Math.cos(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getY() + Math.sin(Math.toDegrees(drive.getPoseEstimate().getHeading()) - 180)*10, drive.getPoseEstimate().getHeading()))
                    .back(2,
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(20))
                    .build();
            lights.green();
        }else if(trajNum == 2){
            calculatedTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(4)
                    .splineTo(new Vector2d(-40, 14), Math.toRadians(180))
                    .build();
            lights.blue();
        }else if(trajNum == 3){
            calculatedTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-36,14, Math.toRadians(135)))
                    .build();
            lights.green();
        }
        trajNum = 0;

        dashboardTelemetry.addData("lastError", drive.getLastError());
        dashboardTelemetry.addData("Pos", drive.getPoseEstimate());
        dashboardTelemetry.addData("duration", traj.duration());
        dashboardTelemetry.addData("end Pos", traj.end());
        dashboardTelemetry.update();
    }
}
