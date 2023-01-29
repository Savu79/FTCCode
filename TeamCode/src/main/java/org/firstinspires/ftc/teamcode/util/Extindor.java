package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config

@TeleOp(name="Extindor", group="drive")

public class Extindor extends LinearOpMode {

    DcMotorEx ExtindorSt;
    DcMotorEx ExtindorDr;
    public static int EInchis=0;
    public static int EDeschis=3400;
    int pozESt=EInchis;
    int pozEDr=EInchis;
    public static double Kp=1;
    @Override
    public void runOpMode(){

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ExtindorDr= hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt= hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            ExtindorSt.setPower((pozESt-ExtindorSt.getCurrentPosition())*Kp);
            ExtindorDr.setPower((pozEDr-ExtindorDr.getCurrentPosition())*Kp);

            if(gamepad1.x) {
                pozESt=EInchis;
                pozEDr=EInchis;
            }
            if(gamepad1.a) {
                pozESt=EDeschis;
                pozEDr=EDeschis;
            }
            if(gamepad1.right_stick_button)
            {
                ExtindorSt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ExtindorDr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(gamepad1.right_stick_y!=0)
            {
                pozESt+=-gamepad1.right_stick_y*20;
                pozEDr+=-gamepad1.right_stick_y*20;
            }
            telemetry.addData("ExtindorSt: ", pozESt);
            telemetry.addData("ExtindorDr: ", pozEDr);
            telemetry.addData("ExtindorSt: ", ExtindorSt.getDirection());
            telemetry.addData("ExtindorSt: ", ExtindorSt.getMode());
            telemetry.update();
        }
    }

}
//