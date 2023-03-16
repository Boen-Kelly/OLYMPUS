package org.firstinspires.ftc.teamcode.classes.TestThread;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class TestThread implements Runnable{
    private int var = 0;
    SampleMecanumDrive drive;
    public TestThread(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
    }
    @Override
    public void run() {
        while (!Thread.interrupted()){
            TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
//                    lift.setSlurpPower(1);
//                    pipeline.setPipelines("pole", "pole");
                    })
                    .lineToConstantHeading(new Vector2d(-34, 64.75))
                    .splineToConstantHeading(new Vector2d(-36, 62.75), Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(-34,3, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-34, 7.5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-36,24, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-36,12, Math.toRadians(135)))
                    .build();
            var ++;
        }
    }

    public String toString(){
        return "" + var;
    }
}
