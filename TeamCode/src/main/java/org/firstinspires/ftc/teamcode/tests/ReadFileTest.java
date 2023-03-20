package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

@TeleOp
@Disabled
public class ReadFileTest extends LinearOpMode {
    public void runOpMode(){
        String data = "";
        try {
            File obj = new File("/sdcard/FIRST/nums.txt");
            Scanner scan = new Scanner(obj);

            while (scan.hasNextLine()){
                telemetry.addLine("Reading...");
                telemetry.update();
                data = scan.nextLine();
            }
            scan.close();
        }catch (FileNotFoundException e){
            telemetry.addLine("couldn't read");
            telemetry.update();
            e.printStackTrace();
        }

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("file says", data);
            telemetry.update();
        }
    }
}
