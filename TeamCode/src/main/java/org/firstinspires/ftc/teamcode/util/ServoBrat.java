package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="ServoBrat", group="drive")
public class ServoBrat extends LinearOpMode {

    public static double pozMaxB=1;
    public static double pozMinB=0.5;
    public static int vitBrat=40;

    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
                    poz=pozMaxB;
                }
                else{
                    poz=pozMinB;
                }
            }
            k++;
            sleep(150);
            if (gamepad1.right_stick_y!=0 && poz<=pozMaxB && poz>=pozMinB){
                poz+=gamepad1.right_stick_y/vitBrat;
            }
            if(poz>pozMaxB){
                poz=pozMaxB;
            }
            if(poz<pozMinB){
                poz=pozMinB;
            }
        }
    }
}
//