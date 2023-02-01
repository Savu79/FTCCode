package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp (name="PIDMotor",group="drive")
public class PIDMotor extends LinearOpMode {

    DcMotorEx motor;

    double integralSum;
    public static double Kp=1;
    public static double Ki=0;
    public static double Kd=0;
    public static double imp=80;
    public static double inm=100;
    int poz=0;
    double power;

    ElapsedTime timer= new ElapsedTime();
    private double lastError;

    @Override
    public void runOpMode() throws InterruptedException{
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        motor=hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.right_stick_button && poz>-1 && poz<1940)
            {
                poz-= (int) (inm* gamepad1.right_stick_y);
            }
            if(poz<=-1)
                poz=0;
            if(poz>=1940)
                poz=1930;
            if (gamepad1.a) {
                poz = 400;
            }
            if( gamepad1.y){
                poz=1500;
            }
            power=PIDControl(poz, motor.getCurrentPosition())/imp;
            motor.setPower(power);
            telemetry.addData("Pozitie dorita ", poz);
            telemetry.addData("Pozitie reala ", motor.getCurrentPosition());
            telemetry.addData("Puterea data motorului ", power);
            telemetry.update();

        }
    }
    public double PIDControl(double reference, double state)
    {
        double error=reference-state;
        integralSum+=error*timer.seconds();
        double derivate= (error-lastError)/timer.seconds();
        double output = (error*Kp)+(derivate*Kd)+(integralSum*Ki);
        return output;
    }

}