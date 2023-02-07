package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="test servo", group="test")
public class TestServo extends LinearOpMode {
    public void runOpMode(){
        Servo Intake;
        int b=0;
        Intake=hardwareMap.get(Servo.class, "servo");
        double poz=0;
        waitForStart();
        while(opModeIsActive()) {
            Intake.setPosition(poz);
            telemetry.addData("Pozitie ", Intake.getPosition());
            telemetry.update();
            if(gamepad1.b)
            {
                if(b%2==0){
                    poz=0;
                }
                else {
                    poz=1;
                }
                b++;
                sleep(200);
            }
            if(gamepad1.right_stick_button && poz>=0 && poz<=1)
            {
                poz-=gamepad1.right_stick_y/100;
            }
            if(poz<0)
                poz=0;
            if(poz>1)
                poz=1;
        }
    }
}