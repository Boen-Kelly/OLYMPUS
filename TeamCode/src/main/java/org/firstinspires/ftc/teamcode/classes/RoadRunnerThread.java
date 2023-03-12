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

import java.lang.reflect.Method;

public class RoadRunnerThread implements Runnable{

    public static float interpolator;
    private boolean countTar = false;
    private Trajectory currentTar;
    private SampleMecanumDrive drive;
    private Trajectory traj;
    public Trajectory calculatedTraj;

    private Vector2d targetPos;
    private double targetHeading;

    private TrajectoryBuilder trajBuilder;

    private int trajNum;

    private HardwareMap hardwareMap;

    RRThreadTest auto = new RRThreadTest();

    public static Pose2d poseWithError1 = new Pose2d(0,0,0);

    public RoadRunnerThread(HardwareMap hardwareMap, SampleMecanumDrive drive, Trajectory traj, TrajectoryBuilder trajBuilder, int trajNum){
        this.drive = drive;
        this.traj = traj;
        this.targetPos = targetPos;
        this.targetHeading = targetHeading;
        this.trajBuilder = trajBuilder;
        this.trajNum = trajNum;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void run(){

        ElapsedTime timer = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        timer.reset();

        lightsThread lights = new lightsThread(hardwareMap);
        while(timer.seconds() < traj.duration()-0.2){
            dashboardTelemetry.addData("timer", timer.seconds());
        }
        poseWithError1 = new Pose2d(traj.end().getX() + drive.getLastError().getX(),traj.end().getY() + drive.getLastError().getY(),traj.end().getHeading());



        if(trajNum == 1){
            calculatedTraj = auto.traj1Builder1(drive, poseWithError1).build();
            lights.green();
        }else if(trajNum == 2){
            calculatedTraj = auto.traj1Builder2(drive, poseWithError1).build();
            lights.blue();
        }



        dashboardTelemetry.addData("lastError", drive.getLastError());
        dashboardTelemetry.addData("Pos", drive.getPoseEstimate());
        dashboardTelemetry.addData("path Length", traj.getPath().length());
        dashboardTelemetry.addData("duration", traj.duration());
        dashboardTelemetry.addData("end Pos", traj.end());
        dashboardTelemetry.update();

    }
}
