package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftArm{
    private DcMotor lift1, lift2, arm;
    private CRServo slurper;

    private int height = 0, prevHeight = height;
    private double slurpPower = 0, prevSlurpPower = slurpPower;
    boolean armUp = false;
    String telemetry = "";

    public LiftArm(HardwareMap hardwareMap){
        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        slurper = hardwareMap.get(CRServo.class, "Slurper");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotor.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        arm.setTargetPosition(0);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slurper.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run() {
        while (!Thread.interrupted()){
            if(height != prevHeight && armUp){
                while((lift1.getCurrentPosition() - 15) < height && height < (lift1.getCurrentPosition() + 15)) {
                    if(arm.getCurrentPosition() < -150) {
                        lift1.setTargetPosition(height);
                        lift2.setTargetPosition(lift1.getTargetPosition());
                        lift1.setPower(1);
                        lift2.setPower(lift1.getPower());
                    }

                    arm.setTargetPosition(-1310);
                    arm.setPower(1);
                    telemetry = "armPos: " + arm.getCurrentPosition() + "\nliftPos: " + lift1.getCurrentPosition();
                }
            }else if(height != prevHeight){
                while((lift1.getCurrentPosition() - 15) < height && height < (lift1.getCurrentPosition() + 15)){
                    lift1.setTargetPosition(height);
                    lift2.setTargetPosition(lift1.getTargetPosition());
                    lift1.setPower(1);
                    lift2.setPower(lift1.getPower());

                    arm.setTargetPosition(0);
                    arm.setPower(1);

                    telemetry = "armPos: " + arm.getCurrentPosition() + "\nliftPos: " + lift1.getCurrentPosition();
                }
            }
            prevHeight = height;
            telemetry = "prev: " + prevHeight + "\nHeight: " + height;
        }
    }

    public void lift (int height, boolean armUp){
        if(armUp){
            arm.setTargetPosition(-1310);
            arm.setPower(1);
            while(arm.getCurrentPosition() > -150){

            }
        }else{
            arm.setTargetPosition(0);
            arm.setPower(1);
        }
        lift1.setTargetPosition(height);
        lift2.setTargetPosition(lift1.getTargetPosition());
        lift1.setPower(1);
        lift2.setPower(lift1.getPower());
    }

    public void setSlurpPower(double power){
        slurper.setPower(power);
    }

    public String toString(){
        return telemetry;
    }
}
