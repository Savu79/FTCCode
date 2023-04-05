package org.firstinspires.ftc.teamcode.util;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="TestServoEncoder", group="test")
public class TestSerovEncoder extends LinearOpMode{

    Encoder enc;
    CRServo servo;
    public static double Kp=0.000005;
    public static double Kd=0.00002;
    public static double Ki=0.00002;
    public double pow=0;
    public static double integralSum=0;
    public static double lastError=0;
    public static double derivative=0;
    public static double error=0;
    public static double encoderPoz=0;
    public static double ref=2150;


    //ElapsedTime timer = new ElapsedTime();

    public void runOpMode (){
        servo = hardwareMap.get(CRServo.class, "servo");
        enc = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder"));
       // enc.setDirection(Encoder.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            if (enc.getCurrentPosition() < 2200 && enc.getCurrentPosition() > 2100)
            {
                pow=0;
            }
            else if (enc.getCurrentPosition() < 2100)
            {
                pow=0.7;
            }
            else if (enc.getCurrentPosition() > 2200)
            {
                pow=-0.7;
            }

            /*encoderPoz = enc.getCurrentPosition();

            error = ref - encoderPoz;

            derivative = (error - lastError) / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            pow = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (pow < 0)
            {
                pow=pow-0.1;
            }
            if (pow > 0)
            {
                pow=pow+0.1;
            }

             */

            servo.setPower(pow);

            //lastError=error;

            //timer.reset();

            telemetry.addData("Pozitie ", enc.getCurrentPosition());
            telemetry.addData("putere", pow);
            telemetry.update();
        }
    }
}
