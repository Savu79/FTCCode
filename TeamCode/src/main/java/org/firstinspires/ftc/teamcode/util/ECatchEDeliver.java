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
    public static double pozServoExtindor = pozServoExtindorJos;
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
    int pozMotorBratJos=0;
    int pozMotorBratSus=320;
    double powerBM=1;

    //Brat
    int pozBratSus = 2300;
    int pozBratJos = 0;
    int pozBrat=pozBratJos;
    double powerB=0.7;

    ///Auto
    boolean autoSeg = false;
    boolean PaharSus=false;
    int a=1;

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

            ExtindorDr.setTargetPosition(pozE);
            ExtindorSt.setTargetPosition(pozE);
            ServoExtindor.setPosition(pozServoExtindor);
            Intake.setPosition(pozIntake);
            Puta.setPosition(pozPuta);
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
            if (PaharSus)
            {
                ElapsedTime timerCase2 = new ElapsedTime();
                switch (a){
                    case 1: {
                        Intake.setPosition(pozIntakeInchis);
                        ServoExtindor.setPosition(pozServoExtindorMij);
                        ExtindorDr.setTargetPosition(EInchis);
                        ExtindorSt.setTargetPosition(EInchis);
                        if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy()) a=2;
                        timerCase2.reset();
                    }
                    case 2: {
                        ServoExtindor.setPosition(pozServoExtindorPas);
                        if(timerCase2.milliseconds()>1000)
                            Intake.setPosition(pozIntakePasare);
                        if(timerCase2.milliseconds()>1300)
                        {
                            ExtindorSt.setTargetPosition(EPas);
                            ExtindorDr.setTargetPosition(EPas);
                            if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy())
                            {
                                Puta.setPosition(pozPutaCu);
                                Intake.setPosition(pozIntakeInchis);
                                a=3;

                            }
                        }

                    }
                    case 3: {
                        ExtindorSt.setTargetPosition(EDeschis);
                        ExtindorDr.setTargetPosition(EDeschis);
                        ServoExtindor.setPosition(pozServoExtindor1);
                        a=4;
                    }
                    case 4: {
                        MotorBrat.setPower( Math.sin(Math.toRadians(MotorBrat.getCurrentPosition()/(537.7/360))) * Ksin+0.1);
                    }
            }
            }


        }
    }

}
