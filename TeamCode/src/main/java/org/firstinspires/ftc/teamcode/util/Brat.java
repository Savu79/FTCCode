package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Brat extends LinearOpMode {
    public void runOpMode(){
        waitForStart();
        Servo ServoSt;
        Servo ServoDr;
        double pozSt = 0;
        double pozDr = 1;
        double k=2;
        ElapsedTime timerB = new ElapsedTime();
        int b=0;
        ServoDr = hardwareMap.get(Servo.class,"ServoDr");
        ServoSt = hardwareMap.get(Servo.class,"ServoSt");
        while (opModeIsActive()){
            ServoSt.setPosition(pozSt);
            ServoDr.setPosition(pozDr);
            if (gamepad1.b && timerB.milliseconds()>300) {
                if(b%2==0)
                pozSt = pozDr = 0.5;
                else
                {
                    pozSt=0;
                    pozDr=1;
                }
                b++;
                timerB.reset();
            }
            if(gamepad1.left_stick_y!=0) {
               pozSt += gamepad1.left_stick_y/k;
               pozDr -= gamepad1.left_stick_y/k;
            }
        }
    }
}
