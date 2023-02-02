package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Test Extindor", group= "test")
public class Test_extindor extends LinearOpMode {
    DcMotorEx ExtindorSt;
    DcMotorEx ExtindorDr;
    public static int EInchis=0;
    public static int EDeschis=3550;
    double power=0;
    double powerDr=0;
    double powerSt=0;
    boolean SEPARAT=true;
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_bumper)
            {
                SEPARAT=true;
            }
            if (gamepad1.left_bumper)
            {
                SEPARAT=false;
            }
            if (SEPARAT){
                ExtindorDr.setPower(powerDr);
                ExtindorSt.setPower(powerSt);
                telemetry.addData("powerDr",powerDr);
                telemetry.addData("powerSt",powerSt);
                telemetry.addData("pozDr",ExtindorDr.getCurrentPosition());
                telemetry.addData("pozSt",ExtindorSt.getCurrentPosition());
                telemetry.update();
                if(gamepad1.left_stick_y!=0)
                {
                    powerSt=gamepad1.left_stick_y;
                }
                if (gamepad1.right_stick_y!=0)
                {
                    powerDr=gamepad1.right_stick_y;
                }
                if (gamepad1.right_stick_button)
                {
                    ExtindorDr.setMode(DcMotor.RunMode.RESET_ENCODERS);
                }
                if (gamepad1.left_stick_button)
                {
                    ExtindorSt.setMode(DcMotor.RunMode.RESET_ENCODERS);
                }
            }
            else {
                ExtindorDr.setPower(power);
                ExtindorSt.setPower(power);
                telemetry.addData("power",power);
                telemetry.addData("pozDr",ExtindorDr.getCurrentPosition());
                telemetry.addData("pozSt",ExtindorSt.getCurrentPosition());
                telemetry.update();
                if (gamepad1.right_stick_y!=0)
                {
                    power=gamepad1.right_stick_y;
                }
                if (gamepad1.right_stick_button)
                {
                    ExtindorDr.setMode(DcMotor.RunMode.RESET_ENCODERS);
                    ExtindorSt.setMode(DcMotor.RunMode.RESET_ENCODERS);
                }
            }
        }
    }
}
