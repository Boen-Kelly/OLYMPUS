package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

import java.util.ArrayList;

@TeleOp
public class PoleDistanceCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        DistanceSensor backDist = hardwareMap.get(DistanceSensor.class, "backDist");

        while (pipeline.toString().equals("waiting for start")){
            telemetry.addLine("waiting for OpenCV");
            telemetry.update();
        }

        while(!opModeIsActive() && !isStopRequested()){
            pipeline.setPipelines("pole", "pole");
            pipeline.frontPoleDetector.setColors(true, false, false);
            pipeline.backPoleDetector.setColors(true, false, false);

            telemetry.addLine("Waiting for start");
            telemetry.update();
        }

        double sensorDist = backDist.getDistance(DistanceUnit.INCH);
        double poleWidth = pipeline.getMaxWidth(false);
        ArrayList<Double> kVals = new ArrayList<>();
        double kValAvg = 0;
        double kValTotal = 0;
        double calculatedDist = 0;

        while (opModeIsActive()){
            sensorDist = backDist.getDistance(DistanceUnit.INCH);
            poleWidth = pipeline.getMaxWidth(false);

            if(gamepad1.a){
                kVals.add(sensorDist * poleWidth);
            }

            if(gamepad1.y){
                telemetry.addLine("Calculating");
                telemetry.update();
                for(double i : kVals){
                    kValTotal += i;
                    kValAvg = kValTotal/kVals.size();
                }

                kValTotal = 0;
                telemetry.addLine("All done!");
                telemetry.update();
            }

            calculatedDist = kValAvg/poleWidth;

            telemetry.addData("distance sensor", sensorDist);
            telemetry.addData("pole width", poleWidth);
            telemetry.addData("calculated dist", calculatedDist);
            telemetry.addData("kval", sensorDist * poleWidth);
            telemetry.addData("kAvg", kValAvg);
            telemetry.addData("array size", kVals.size());
            telemetry.update();
        }
    }
}
