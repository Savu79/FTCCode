package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.TestMotorEncoder.Ksin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dalvik.system.DelegateLastClassLoader;

@Config
@TeleOp(name="ECatchEDeliver", group= "test")
public class ECatchEDeliver extends LinearOpMode {

    Servo Intake;
    Servo ServoExtindor;
    Servo Puta;
    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    DcMotorEx Brat;
    DcMotorEx MotorBrat;

    ///timer
    public ElapsedTime timerBrat= new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timerCase2 = new ElapsedTime();

    ////Intake
    public static double pozIntakeDeschis = 0.17;
    public static double pozIntakeInchis = 0;
    public static double pozIntakePasare = 0.1;
    double pozIntake;

    ///// ServoExtindor
    public static double pozServoExtindorMij = 0.46;
    public static double pozServoExtindor1 = 0.06;
    public static double pozServoExtindor2 = 0.10;
    public static double pozServoExtindor3 = 0.15;
    public static double pozServoExtindor4 = 0.21;
    public static double pozServoExtindor5 = 0.25;
    public static double pozServoExtindorPas = 0.63;
    public static double pozServoExtindorJos = 0;
    public static double pozServoExtindor = pozServoExtindor1;
    public static double kx = 20;

    ///Extindor
    public static double powerE = 1;
    int pozE = 0;
    int EInchis = 0;
    int EDeschis = 2700;
    int EPas =1250;

    //Puta
    double pozPuta;
    double pozPutaCu=0.14;
    double pozaPutaFara=0;

    //MotorBrat
    boolean directieMotorBratSus=false;
    double Kcos=-0.41;
    double Ksin=-0.7;

    //Brat
    int pozBratSus = 2300;
    int pozBratJos = 0;
    int pozBrat=pozBratJos;
    double powerB=0.7;

    public static double a=0;

    public static double Kp=0.01;
    public static double Ki=0.01;
    public static double Kd=0.0005;

    int reference=pozBratJos;
    int lastReference=reference;

    int error;
    int lastError;

    double integralSum=0;
    public static double integralSumLim=0;

    double derivative;
    float currentFilterEstimate;
    float previousFilterEstimate;

    ///Auto
    boolean autoSeg = false;
    boolean PaharSus=false;
    boolean DejaSus=false;
    int SWITCHSUS=1;

    public void runOpMode (){
        waitForStart();

        Intake       =hardwareMap.get(Servo.class, "Intake");
        ServoExtindor=hardwareMap.get(Servo.class, "ServoExtindor");
        ExtindorDr   =hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt   =hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        Puta         =hardwareMap.get(Servo.class, "Puta");
        Brat         =hardwareMap.get(DcMotorEx.class, "Brat");
        MotorBrat    =hardwareMap.get(DcMotorEx.class, "MotorBrat");


        Brat.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);

        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Brat.setTargetPosition(pozBratJos);
        ExtindorDr.setTargetPosition(EInchis);
        ExtindorSt.setTargetPosition(EInchis);
        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtindorDr.setPower(powerE);
        ExtindorSt.setPower(powerE);
        Brat.setPower(powerB);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {

            if(directieMotorBratSus)
                MotorBrat.setPower( Math.sin(Math.toRadians(MotorBrat.getCurrentPosition()/(537.7/360))) * Ksin+0.1);
            else
                MotorBrat.setPower(Math.cos(Math.toRadians( MotorBrat.getCurrentPosition()/(357.7/360))) * Kcos-0.2);

            ExtindorDr.setTargetPosition(pozE);
            ExtindorSt.setTargetPosition(pozE);

            ServoExtindor.setPosition(pozServoExtindor);

            Intake.setPosition(pozIntake);

            Puta.setPosition(pozPuta);

            Brat.setPower((error*Kp) + (integralSum*Ki) + (derivative*Kd));

            lastReference=reference;
            reference=pozBrat;

            error=pozBrat-Brat.getCurrentPosition();

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

            telemetry.addData("PozitieIntake ", Intake.getPosition());
            telemetry.addData("PozitieServoExtindor", ServoExtindor.getPosition());
            telemetry.addData("PozitieExtindor", ExtindorDr.getCurrentPosition());
            telemetry.update();

            //////Mutare Extindor
            if (gamepad2.left_stick_y!=0 && pozE<=EDeschis && pozE>=EInchis)
                pozE-=gamepad2.left_stick_y*kx;
            if (pozE<EInchis)
                pozE=EInchis;
            if (pozE>EDeschis)
                pozE=EDeschis;

            ////Mutare Intake
            if (gamepad1.right_stick_y != 0 && pozIntake >= 0 && pozIntake <= 1) {
                pozIntake -= gamepad1.right_stick_y / 100;
            }
            if(pozIntake<0)
                pozIntake=0;
            if(pozIntake>1)
                pozIntake=1;

            ///Mutare ServoExtindor
            if (gamepad1.left_stick_y != 0 && pozServoExtindor >= 0 && pozServoExtindor <= 1) {
                pozServoExtindor -= gamepad1.left_stick_y / 100;
            }
            if(pozServoExtindor<0)
                pozServoExtindor=0;
            if(pozServoExtindor>1)
                pozServoExtindor=1;

            ///autoSeg
            if(gamepad2.start) PaharSus=true;
            if (PaharSus && !DejaSus)
            {
                switch (SWITCHSUS){
                    case 1:
                        pozIntake=pozIntakeInchis;
                        pozServoExtindor=pozServoExtindorMij;
                        pozE=EInchis;
                        if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy()) SWITCHSUS++;
                        timerCase2.reset();
                        break;

                    case 2:
                        pozServoExtindor=pozServoExtindorPas;
                        if(timerCase2.milliseconds()>1000)//////////////
                            pozIntake=pozIntakePasare;
                        if(timerCase2.milliseconds()>1300)
                        {
                            pozE=EPas;
                            if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy())
                            {
                                pozPuta= pozPutaCu;
                                pozIntake=pozIntakeInchis;
                                SWITCHSUS++;
                            }
                        }
                        break;

                    case 3:
                        pozE=EDeschis;
                        pozServoExtindor=pozServoExtindor1;
                        SWITCHSUS++;
                        timerBrat.reset();
                        break;
                    case 4:
                        directieMotorBratSus=true;
                        if(timerBrat.milliseconds()>500) {
                            pozBrat = pozBratSus;
                            DejaSus=true;
                        }
            }
            }


        }
    }

}
