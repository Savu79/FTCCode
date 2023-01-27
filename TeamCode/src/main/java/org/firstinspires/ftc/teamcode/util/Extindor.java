package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Extindor", group="drive")
public class Extindor extends LinearOpMode {
    DcMotorEx ExtindorSt;
    DcMotorEx ExtindorDr;
    int EInchis=0;
    int EDeschis=3400;
    int pozESt=EInchis;
    int pozEDr=EInchis;
    public void runOpMode(){

        ExtindorDr= hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt= hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        ExtindorSt.setTargetPosition(EInchis);
        ExtindorDr.setTargetPosition(EInchis);
        ExtindorSt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtindorDr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtindorDr.setPower(0.4);
        ExtindorSt.setPower(0.4);

        while(opModeIsActive())
        {
            ExtindorSt.setTargetPosition(pozESt);
            ExtindorDr.setTargetPosition(pozEDr);
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