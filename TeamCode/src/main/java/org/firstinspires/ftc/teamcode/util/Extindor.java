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
@Config

@TeleOp(name="Extindor", group="drive")

public class Extindor extends LinearOpMode {

    DcMotorEx ExtindorSt;
    DcMotorEx ExtindorDr;
    public static int EInchis=0;
    public static int EDeschis=3550;
    public static double a=0;
    int errorSt;
    int errorDr;
    double integralSumDr;
    double integralSumSt;
    double derivativeSt;
    double derivativeDr;
    int lastErrorSt=0;
    int lastErrorDr=0;
    int pozESt=EInchis;
    int pozEDr=EInchis;
    int reference=EInchis;
    int lastReference=reference;

    float currentFilterEstimateSt;
    float currentFilterEstimateDr;
    float previousFilterEstimateSt;
    float previousFilterEstimateDr;
    public static double Kp=0.01;
    public static double Ki=0;
    public static double Kd=0;
    public static double integralSumLim=0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ExtindorDr= hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt= hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        ElapsedTime timerD = new ElapsedTime();
        while(opModeIsActive())
        {
            lastReference=reference;
            reference= pozEDr;

            errorSt=pozESt-ExtindorSt.getCurrentPosition();
            errorDr=pozEDr-ExtindorDr.getCurrentPosition();

            currentFilterEstimateDr = ((float)a * previousFilterEstimateDr) + (1-(float)a) * (errorDr-lastErrorDr);
            currentFilterEstimateSt = ((float)a * previousFilterEstimateSt) + (1-(float)a) * (errorSt-lastErrorSt);

            integralSumSt = integralSumSt + (errorSt * timer.seconds());
            integralSumDr = integralSumDr + (errorDr * timer.seconds());

            derivativeSt = currentFilterEstimateSt / timer.seconds();
            derivativeDr = currentFilterEstimateDr / timer.seconds();

            lastErrorSt=errorSt;
            lastErrorDr=errorDr;

            previousFilterEstimateDr = currentFilterEstimateDr;
            previousFilterEstimateSt = currentFilterEstimateSt;

            timer.reset();

            if (integralSumSt > integralSumLim)
                integralSumSt=integralSumLim;
            if (integralSumDr > integralSumLim)
                integralSumDr=integralSumLim;
            if (integralSumSt < -integralSumLim)
                integralSumSt= -integralSumLim;
            if (integralSumDr < -integralSumLim)
                integralSumDr= -integralSumLim;
            if(reference!=lastReference)
            {
                integralSumSt=0;
                integralSumDr=0;
            }
            ExtindorSt.setPower(  (errorSt*Kp)  +  (integralSumSt*Ki)  + (derivativeSt*Kd)  );
            ExtindorDr.setPower(  (errorDr*Kp)  +  (integralSumDr*Ki)  + (derivativeDr*Kd)  );

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

            if(gamepad1.left_stick_y!=0)
                pozEDr+=-gamepad1.right_stick_y*20;

            telemetry.addData("ExtindorSt power: ", ExtindorSt.getPower());
            telemetry.addData("ExtindorDr power: ", ExtindorDr.getPower());
            telemetry.addData("pozSt" ,pozESt*0.001);
            telemetry.addData("pozDr", pozEDr*0.001);
            telemetry.addData("ExtindorSt pozitie(ticks): ", (double)ExtindorSt.getCurrentPosition()/1000);
            telemetry.addData("ExtindorDr pozitie(ticks): ", (double)ExtindorDr.getCurrentPosition()/1000);

            telemetry.update();
        }
    }

}