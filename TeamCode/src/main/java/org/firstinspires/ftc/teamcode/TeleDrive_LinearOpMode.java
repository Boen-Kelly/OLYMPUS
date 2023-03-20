
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.TapeMeasure;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

@TeleOp(name = "TeleDrive LinearOpMode", group = "")
@Disabled
public class TeleDrive_LinearOpMode extends LinearOpMode {
    private DatagramSocket socket;
    private boolean canRunGamepadThread;
    private Thread gamepadHandler;
    public String telem = "";

    private void startGamepadHandlerThread() {
        telemetry.setAutoClear(true);
        gamepadHandler = new Thread(new Runnable() {
            @Override
            public void run() {
                while (canRunGamepadThread) {
                    String gamepadAction = "";
                    try {
                        byte[] buffer = new byte[1024];
                        DatagramPacket response = new DatagramPacket(buffer, buffer.length);
                        socket.receive(response);
                        gamepadAction = new String(buffer, 0, response.getLength());
                    } catch (Exception ignore) {

                    }

                    if (!gamepadAction.isEmpty()) {
                        if(gamepadAction.contains("E-STOP")) {
                            requestOpModeStop();
                        }
                        if (gamepadAction.contains("G1")) {
                            telem = gamepadAction;
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("DU")) {
                                    gamepad1.dpad_up = true;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DD")) {
                                    gamepad1.dpad_down = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DL")) {
                                    gamepad1.dpad_left = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DR")) {
                                    gamepad1.dpad_right = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.back = false;
                                }
                            }
                            if (gamepadAction.contains("_LX_")) {
                                gamepad1.left_stick_x = Float.parseFloat(gamepadAction.replace("G1_LX_", ""));
                                telem = gamepadAction;
                            }
                            if (gamepadAction.contains("_LY_")) {
                                gamepad1.left_stick_y = Float.parseFloat(gamepadAction.replace("G1_LY_", ""));
                                telem = gamepadAction;
                            }
                            if (gamepadAction.contains("_RX_")) {
                                gamepad1.right_stick_x = Float.parseFloat(gamepadAction.replace("G1_RX_", ""));
                                telem = gamepadAction;
                            }
                            if (gamepadAction.contains("_RY_")) {
                                gamepad1.right_stick_y = Float.parseFloat(gamepadAction.replace("G1_RY_", ""));
                                telem = gamepadAction;
                            }
                        }
                    }
                }
                gamepadHandler.interrupt();
            }
        });
        gamepadHandler.setName("Gamepad Handler Thread");
        gamepadHandler.setPriority(Thread.NORM_PRIORITY);
        gamepadHandler.start();
    }

    @Override
    public void runOpMode() {
        String address = "192.168.43.1"; //Check "Program and Manage" tab on the Driver Station and verify the IP address
        int port = 11039; //Change as needed
        canRunGamepadThread = false;

        try {
            this.socket = new DatagramSocket(port);
        } catch (Exception ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Connect your server to " + address + ":" + port, "");
        telemetry.update();
        canRunGamepadThread = true;

        startGamepadHandlerThread();


        //CUSTOM CODE GOES HERE
        final double INCREMENT = 0.025;
        final double HOR_MAX_POS = 1.0;
        final double HOR_MIN_POS = .15;
        final double VER_MAX_POS = .67;
        final double VER_MIN_POS = .25;

        Servo servo;
        servo = hardwareMap.get(Servo.class, "horizontal");
        Servo servo2;
        servo2 = hardwareMap.get(Servo.class, "vertical");
        double position = .72;
        double position2 = .43;
        canRunGamepadThread = true;

        DcMotor bl,br,fl,fr;
        double fdist, bdist, ldist, rdist;
        double xPower = 0.0, prevXPower = xPower;
        double yPower = 0.0, prevYPower = yPower;
        double fMaxVal = .5, bMaxVal = .5;
        double minOne, minTwo;

        DistanceSensor back, front, left, right;

        DcMotor leftMotor;
        DcMotor rightMotor;

        leftMotor = hardwareMap.get(DcMotor.class, "l");
        rightMotor = hardwareMap.get(DcMotor.class, "r");
        back = hardwareMap.get(DistanceSensor.class, "back");
        front = hardwareMap.get(DistanceSensor.class, "front");
        left = hardwareMap.get(DistanceSensor.class, "left");
        right = hardwareMap.get(DistanceSensor.class, "right");

        waitForStart();
        while (opModeIsActive()){
            fdist = front.getDistance(DistanceUnit.CM);
            bdist = back.getDistance(DistanceUnit.CM);
            ldist = left.getDistance(DistanceUnit.CM);
            rdist = right.getDistance(DistanceUnit.CM);

            if (gamepad1.dpad_left) {
                position = position - INCREMENT;
            }
            if (position < HOR_MIN_POS) {
                position = HOR_MIN_POS;
            }
            if (gamepad1.dpad_right) {
                position = position + INCREMENT;
            }
            if (position > HOR_MAX_POS) {
                position = HOR_MAX_POS;
            }
            if (gamepad1.dpad_down) {
                position2 = position2 - INCREMENT;
            }
            if (position2 < VER_MIN_POS) {
                position2 = VER_MIN_POS;
            }
            if (gamepad1.dpad_up) {
                position2 = position2 + INCREMENT;
            }
            if (position2 > VER_MAX_POS) {
                position2 = VER_MAX_POS;
            }

            servo.setPosition(position);
            servo2.setPosition(position2);

            xPower = -(gamepad1.left_stick_x * .2) + (prevXPower * .8);
            yPower = -(gamepad1.left_stick_y * .2) + (prevYPower * .8);
            if(yPower > 0) {
                yPower = yPower * fMaxVal;
            }else if(yPower < 0){
                yPower = yPower * bMaxVal;
            }

            if(fdist < 30){
                minOne = Math.min(ldist, rdist);
                minTwo = Math.min(minOne, fdist);
                fMaxVal = 1 * minTwo/30;
                bMaxVal = 1;
            }else if(bdist < 30){
                minOne = Math.min(ldist, rdist);
                minTwo = Math.min(minOne, bdist);
                fMaxVal = 1;
                bMaxVal = 1 * minTwo/30;
            }else if(ldist < 30 || rdist < 30){
                minOne = Math.min(ldist, rdist);
                bMaxVal = 1 * minOne/30;
                fMaxVal = 1 * minOne/30;
            }else{
                fMaxVal = 1;
                bMaxVal = 1;
            }

            leftMotor.setPower((yPower + xPower));
            rightMotor.setPower((yPower - xPower));

            telemetry.addData("Distances in", "CM");
            telemetry.addData("Back sensor", back.getDistance(DistanceUnit.CM));
            telemetry.addData("Front sensor", front.getDistance(DistanceUnit.CM));
            telemetry.addData("Left sensor", left.getDistance(DistanceUnit.CM));
            telemetry.addData("Right sensor", right.getDistance(DistanceUnit.CM));
            telemetry.addData("yPower", yPower);
            telemetry.addData("xPower", xPower);
            telemetry.addData("Telemetry", telem);
            telemetry.update();

            prevXPower = xPower;
            prevYPower = yPower;
        }
    }
}