package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autos.BLUE.RightParkCone;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class RoadRunnerThread implements Runnable{

    public static float interpolator;
    private boolean countTar = false;
    private Trajectory currentTar;
    private SampleMecanumDrive drive;
    private Trajectory traj;

    RightParkCone auto = new RightParkCone();

    public RoadRunnerThread(HardwareMap hardwareMap, SampleMecanumDrive drive, Trajectory traj){
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void run(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (!Thread.interrupted()) {
            dashboardTelemetry.addData("lastError", drive.getLastError());
            dashboardTelemetry.addData("Pos", drive.getPoseEstimate());
            dashboardTelemetry.addData("path Length", traj.getPath().length());
            dashboardTelemetry.addData("duration", traj.duration());
            dashboardTelemetry.addData("end Pos", traj.end());
            dashboardTelemetry.update();
        }
    }
}
