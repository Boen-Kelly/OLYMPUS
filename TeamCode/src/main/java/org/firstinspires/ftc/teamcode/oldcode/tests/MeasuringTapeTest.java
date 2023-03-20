package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.TapeMeasure;


@TeleOp
@Disabled
public class MeasuringTapeTest extends LinearOpMode {
    public void runOpMode(){
        TapeMeasure tape = new TapeMeasure(hardwareMap, gamepad1, gamepad2, TapeMeasure.Side.BLUE);
        boolean toggle = false;

        Thread tapeThread = new Thread(tape);

        telemetry.addData("Tape Telemetry", tape);
        telemetry.update();

        waitForStart();
        tapeThread.start();
        while (opModeIsActive()){
            telemetry.addData("Tape Telemetry", tape);
            telemetry.update();
        }
        tapeThread.interrupt();
    }
}
