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

@TeleOp (name= "TestPIDBrat", group="test")
@Config
public class TestPIDBrat extends LinearOpMode {

    public static int BInchis=0;
    public static int BDeschis=0;

    public static double a=0;

    public static double Kp=0;
    public static double Ki=0;
    public static double Kd=0;

    int reference=BInchis;
    int lastReference=reference;

    int error;
    int lastError;

    double integralSum=0;
    public static double integralSumLim=0;

    double derivative;
    float currentFilterEstimate;
    float previousFilterEstimate;

    int pozB;

    ElapsedTime timer = new ElapsedTime();

    DcMotorEx Brat;

    @Override
    public void runOpMode(){

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Brat=hardwareMap.get(DcMotorEx.class, "Brat");

        Brat.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            lastReference=reference;
            reference=pozB;

            error=pozB-Brat.getCurrentPosition();

            currentFilterEstimate=(float)a*previousFilterEstimate + (1-(float)a*(error-lastError));

            integralSum+=error*timer.seconds();

            derivative=currentFilterEstimate/timer.seconds();

            previousFilterEstimate=currentFilterEstimate;

            timer.reset();

            if (integralSum > integralSumLim)
                integralSum=integralSumLim;
            if (integralSum < -integralSumLim)
                integralSum=-integralSumLim;

            if(reference!=lastReference)
                integralSum=0;

            if(gamepad1.x) pozB=BInchis;
            if(gamepad1.a) pozB=BDeschis;

            if(gamepad1.right_stick_button)
                Brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(gamepad1.right_stick_y!=0)
                pozB=(int)-gamepad1.right_stick_y*20;

            Brat.setPower((error*Kp) + (integralSum*Ki) + (derivative*Kd));

            telemetry.addData("ExtindorSt power: ", Brat.getPower());
            telemetry.addData("pozSt" ,pozB*0.001);
            telemetry.addData("ExtindorSt pozitie(ticks): ", (double)Brat.getCurrentPosition()/(double)1000);
            telemetry.update();
        }
    }

}
