package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="Test Brat ENcoder", group="test")
public class TestMotorEncoder extends LinearOpMode {
    DcMotorEx motor;

    int poz=0;
    public void runOpMode(){
        motor=hardwareMap.get(DcMotorEx.class, "motor");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setPower(0.2);
        while(opModeIsActive())
        {
            motor.setTargetPosition(poz);

            if (gamepad1.right_stick_y!=0 && poz<=300 && poz>=0){
                poz-=gamepad1.right_stick_y*20;
            }
            if (poz<0)
            {
                poz=0;
            }
            if (poz>300)
            {
                poz=300;
            }
            if(gamepad1.a)
                poz=0;
            if(gamepad2.b)
                poz=300;
        }
    }

}
