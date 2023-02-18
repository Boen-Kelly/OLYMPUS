package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class RoadRunnerThread implements Runnable{

    public static float interpolator;
    private boolean countTar = false;
    private Trajectory currentTar;
    private SampleMecanumDrive drive;

    public RoadRunnerThread(HardwareMap hardwareMap, SampleMecanumDrive drive){
        this.drive = drive;
    }

    @Override
    public void run(){
        while (!Thread.interrupted()) {
            if (countTar){
                countTajectory();
                countTar = false;
            }
        }
    }
    public void setTarVariables(Trajectory currentTar, boolean countTar){
        this.currentTar = currentTar;
        this.countTar = countTar;
    }
    public void countTajectory(){
    }
    public void calculateNextTar(float errorX, float errorY, Pose2d endPos, Pose2d nextPos){

    }
}
