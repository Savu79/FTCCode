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

    public static int EInchis=0;
    public static int EDeschis=3550;
    double power=0;
    double powerDr=0;
    double powerSt=0;
    boolean SEPARAT=true;
    public void runOpMode(){
        DcMotorEx ExtindorSt;
        DcMotorEx ExtindorDr;
        ExtindorSt= hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        ExtindorDr= hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_bumper)
            {
                SEPARAT=true;
                telemetry.clear();
            }
            if (gamepad1.left_bumper)
            {
                SEPARAT=false;
                telemetry.clear();
            }
            if (SEPARAT){
                ExtindorDr.setPower(powerDr);
                ExtindorSt.setPower(powerSt);
                telemetry.addData("powerDr",powerDr);
                telemetry.addData("powerSt",powerSt);
                telemetry.addData("pozDr",ExtindorDr.getCurrentPosition());
                telemetry.addData("pozSt",ExtindorSt.getCurrentPosition());
                telemetry.update();
                powerSt=gamepad1.left_stick_y;
                powerDr=gamepad1.right_stick_y;
                if (gamepad1.right_stick_button)
                {
                    ExtindorDr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (gamepad1.left_stick_button)
                {
                    ExtindorSt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            else {
                ExtindorDr.setPower(power);
                ExtindorSt.setPower(power);
                telemetry.addData("power",power);
                telemetry.addData("pozDr",ExtindorDr.getCurrentPosition());
                telemetry.addData("pozSt",ExtindorSt.getCurrentPosition());
                telemetry.update();
                power=gamepad1.right_stick_y;
                if (gamepad1.right_stick_button)
                {
                    ExtindorDr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ExtindorSt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
        }
    }
}
