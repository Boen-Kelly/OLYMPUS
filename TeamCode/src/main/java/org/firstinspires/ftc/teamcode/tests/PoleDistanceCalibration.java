package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

import java.util.ArrayList;

@TeleOp
@Config
public class PoleDistanceCalibration extends LinearOpMode {
    public static double exposure = 25;
    public static int gain = 1;
    public static int WB = 5000;
    @Override
    public void runOpMode() throws InterruptedException {
        AutoAlignPipeline pipeline = new AutoAlignPipeline(hardwareMap, "Webcam 2");

        DistanceSensor frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");

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

        double sensorDist = frontDist.getDistance(DistanceUnit.INCH);
        double poleWidth = pipeline.getMaxWidth(true);
        ArrayList<Double> kVals = new ArrayList<>();
        double kValAvg = 0;
        double kValTotal = 0;
        double calculatedDist = 0;

        while (opModeIsActive()){
            sensorDist = frontDist.getDistance(DistanceUnit.INCH);
            poleWidth = pipeline.getMaxWidth(true);

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

            pipeline.setCamVals(exposure,gain,WB);

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
