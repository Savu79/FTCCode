package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoBrat", group="drive")
public class ServoBrat extends LinearOpMode {
    public void runOpMode(){
        Servo BratServo;
        BratServo=hardwareMap.get(Servo.class, "BratServo");
        int k=0;
        double poz=1;
        waitForStart();
        while(opModeIsActive()){
            BratServo.setPosition(poz);
            telemetry.addData("Pozitie ", BratServo.getPosition());
            telemetry.update();
            if (gamepad1.a){
                if (k%2==0){
                    poz=0;
                }
                else{
                    poz=0.5;
                }
            }
            k++;
            sleep(150);
            if (gamepad1.right_stick_y!=0 && poz<=1 && poz>=0.5){
                poz+=gamepad1.right_stick_y/100;
            }
            if(poz>1){
                poz=1;
            }
            if(poz<0.5){
                poz=0.5;
            }
        }
    }
}
