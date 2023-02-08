package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="Test Brat ENcoder", group="test")
@Config
public class TestMotorEncoder extends LinearOpMode {
    DcMotorEx motor;
    public static double Ksin=-0.7;
    public static double Kcos=-0.41;
    boolean SIN=true;
    public void runOpMode(){

        motor=hardwareMap.get(DcMotorEx.class, "motor");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive())
        {
            if(gamepad1.b)
                SIN=true;
            if(gamepad1.a)
                SIN=false;

            if(SIN)
                motor.setPower( Math.sin(Math.toRadians(motor.getCurrentPosition()/(537.7/360))) * Ksin+0.1);
            else {
                motor.setPower(Math.cos(Math.toRadians( motor.getCurrentPosition()/(357.7/360))) * Kcos-0.2);
            }
            if(gamepad1.right_stick_button) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Poz encoder: ", motor.getCurrentPosition());
            telemetry.addData("Poz encoder (grade): ", motor.getCurrentPosition()/(537.7/360));
            telemetry.addData("Power: ", motor.getPower());
            telemetry.update();
        }
    }

}
