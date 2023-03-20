package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp
@Disabled
public class ColorSensorTest extends LinearOpMode {
    public void runOpMode() {
        NormalizedColorSensor color = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        float [] hsvValues = new float [3];

        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            color.setGain(3);
            Color.colorToHSV(color.getNormalizedColors().toColor(), hsvValues);

            if(hsvValues[1] > .4){
                intake.setPower(-1);
            }else{
                intake.setPower(-1);
            }

            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.update();
        }
    }
}
