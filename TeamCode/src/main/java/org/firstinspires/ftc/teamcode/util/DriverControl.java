package org.firstinspires.ftc.teamcode.util;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp (name= "Driver Control", group = "drive")
public class DriverControl extends LinearOpMode {


    Servo Intake;
    Servo ServoExtindor;
    Servo Puta;
    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    DcMotorEx Brat;
    DcMotorEx MotorBrat;

    //viteza
    double vit=1;

    ///timer
    ElapsedTime timerBrat= new ElapsedTime();
    ElapsedTime timerCase1 = new ElapsedTime();
    ElapsedTime timerCase2 = new ElapsedTime();
    ElapsedTime timerCase3= new ElapsedTime();
    ElapsedTime timerCase4= new ElapsedTime();
    ElapsedTime timerY = new ElapsedTime();
    ElapsedTime timerJos = new ElapsedTime();

    //valori timere
    public static int zero=1000;
    public static int unu=1000;
    public static int doi=1400;
    public static int trei=1700;
    public static int patru=500;
    public static int cinci=900;
    public static int sase=1500;
    public static int cleste=400;

    ////Intake
    static double pozIntakeDeschis = 0.17;
    static double pozIntakeInchis = 0;
    public static double pozIntakePasare = 0.1;
    double pozIntake;

    ///// ServoExtindor
    static double pozServoExtindorMij = 0.46;
    static double pozServoExtindor1 = 0.06;
    static double pozServoExtindor2 = 0.10;
    static double pozServoExtindor3 = 0.15;
    static double pozServoExtindor4 = 0.21;
    static double pozServoExtindor5 = 0.25;
    public static double pozServoExtindorPas = 0.67;
    static double pozServoExtindorJos = 0;
    static double pozServoExtindor = pozServoExtindor1;


    ///Extindor
    static double powerE = 1;
    int EBlocat;
    int pozE = 0;
    int EInchis= 0;
    int EDeschis = 2700;
    public int EPas =1250;
    static double kx = 100;

    //Puta
    double pozPutaCu=0.87;
    double pozPutaFara=1;
    double pozPuta=pozPutaFara;

    //MotorBrat
    boolean directieMotorBratSus=false;
    public static double Kcos=-0.41;
    public static double PowerSin=0.1;
    public static double PowerCos=0.2;
    public static double Ksin=-0.7;
    boolean GPy=true;

    //Brat
    int pozBratSus = 2200;
    int pozBratJos = 0;
    int pozBrat=pozBratJos;
    public static double powerB=0.6;

    /*static double a=0;

    static double Kp=0.01;
    static double Ki=0.01;
    static double Kd=0.0005;

    int reference=pozBratJos;
    int lastReference=reference;

    int error;
    int lastError;

    double integralSum=0;
    public static double integralSumLim=0;

    double derivative;
    float currentFilterEstimate;
    float previousFilterEstimate;*/

    ///Auto
    boolean PaharSus=false;
    boolean DejaSus=false;
    boolean PaharJos=false;
    boolean deMaiMulte=false;
    int SWITCHSUS=1;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

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
        Brat.setTargetPosition(pozBratJos);
        Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Brat.setPower(powerB);
        ExtindorDr.setTargetPosition(EInchis);
        ExtindorSt.setTargetPosition(EInchis);
        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setPower(powerE);
        ExtindorSt.setPower(powerE);

        waitForStart();


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeIsActive()){

            //viteze
            if(gamepad1.dpad_left) vit=3;
            if(gamepad1.dpad_up) vit=2;
            if(gamepad1.dpad_right) vit=1;

            //condus
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/vit,
                            -gamepad1.right_stick_x/vit,
                            -gamepad1.left_stick_x/vit
                    )
            );
            drive.update();
            Pose2d poseEstimate=drive.getPoseEstimate();


            //power-pozitii

            ExtindorDr.setTargetPosition(pozE);
            ExtindorSt.setTargetPosition(pozE);

            Brat.setTargetPosition(pozBrat);

            ServoExtindor.setPosition(pozServoExtindor);

            Intake.setPosition(pozIntake);

            Puta.setPosition(pozPuta);

            MotorBrat.setPower(COSSIN(directieMotorBratSus));

            //Mutare Extindor -GP1
            if (gamepad1.right_trigger!=0 && pozE<=EDeschis)
                pozE+=gamepad1.right_trigger*kx;
            if(gamepad1.left_trigger!=0 && pozE>=EInchis)
                pozE-=gamepad1.left_trigger*kx;
            if (pozE<EInchis)
                pozE=EInchis;
            if (pozE>EDeschis)
                pozE=EDeschis;

            //Mutare ServoExtindor -GP2
            if (gamepad2.right_stick_y != 0 && pozServoExtindor >= 0 && pozServoExtindor <= 1)
                pozServoExtindor -= gamepad2.right_stick_y / 80;
            if(pozServoExtindor<0)
                pozServoExtindor=0;
            if(pozServoExtindor>pozServoExtindorMij)
                pozServoExtindor=pozServoExtindorMij;

            //Mutare Brat -GP2
            if (gamepad2.right_trigger!=0 && pozBrat<=pozBratSus){
                pozBrat+=gamepad2.right_trigger*80;
            }

            if (gamepad2.left_trigger!=0 && pozBrat>=pozBratJos)
            {
                pozBrat-=gamepad2.left_trigger*80;
            }


            if(pozBrat<pozBratJos)
                pozBrat=pozBratJos;
            if(pozBrat>pozBratSus)
                pozBrat=pozBratSus;

            //Mutare MotorBrat -GP2
            if(gamepad2.y && timerY.milliseconds()>300)
            {
                if(GPy){
                    directieMotorBratSus=true;
                }

                else {
                    directieMotorBratSus=false;
                }
                GPy=!GPy;
                timerY.reset();
            }

            //Mutare Intake -GP2
            if(gamepad2.a)
                pozIntake= pozIntakeDeschis;
            if(gamepad2.b)
                pozIntake=pozIntakeInchis;

            //Mutare Puta -GP1
            if(gamepad1.b)
                pozPuta=pozPutaFara;
            if(gamepad1.a)
                pozPuta=pozPutaCu;

            //auto Pahar Sus
            if(gamepad2.right_bumper) {
                timerCase1.reset();
                PaharSus=true;
            }
            if (PaharSus && !DejaSus)
                autoSegSus();


            //auto Pahar Jos
            if(gamepad2.left_bumper) PaharJos=true;
            if (PaharJos && DejaSus)
                autoSegJos();


            telemetry.addData("Pozitie robot X: ", toCm(poseEstimate.getX()));
            telemetry.addData("Pozitie robot Y: ", toCm(poseEstimate.getY()));
            telemetry.addData("Heading robot: ", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Brat: ", Brat.getCurrentPosition());
            telemetry.addData("Putere Brat: ", Brat.getPower());
            telemetry.addData("Extindor: ", ExtindorDr.getCurrentPosition());
            telemetry.addData("Pozitie MotorBrat", MotorBrat.getCurrentPosition());
            telemetry.addData("Power MotorBrat ", MotorBrat.getCurrentPosition());
            telemetry.addData("EBlocat: ", EBlocat);
            telemetry.update();
        }
    }

    /*double PIDControlBrat(int pozBrat)
    {
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
        return ((error*Kp) + (integralSum*Ki) + (derivative*Kd));
    }*/
    double COSSIN(boolean directieMotorBratSus)
    {
        if(directieMotorBratSus)
            return(Math.sin(Math.toRadians(MotorBrat.getCurrentPosition()/(537.7/360))) * Ksin+PowerSin);
        else
            return(Math.cos(Math.toRadians(MotorBrat.getCurrentPosition()/(357.7/360))) * Kcos-PowerCos);
    }
    void autoSegSus(){
        switch (SWITCHSUS){
            case 1:
                EBlocat=ExtindorDr.getCurrentPosition();
                pozPuta=pozPutaFara;
                pozIntake=pozIntakeInchis;
                pozServoExtindor=pozServoExtindorMij;
                if(timerCase1.milliseconds()>zero)
                {
                    pozE=EInchis;
                    if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy()) SWITCHSUS++;
                    timerCase2.reset();
                }
                break;

            case 2:
                if(timerCase2.milliseconds()>unu)
                    pozServoExtindor=pozServoExtindorPas;
                if(timerCase2.milliseconds()>doi)
                {
                    pozIntake=pozIntakePasare;
                    pozE=EPas;
                }
                if(timerCase2.milliseconds()>trei)
                {
                    if(!ExtindorDr.isBusy() && !ExtindorSt.isBusy())
                    {
                        pozPuta= pozPutaCu;
                        pozIntake=pozIntakeInchis;
                        SWITCHSUS++;
                        timerCase3.reset();

                    }
                }
                break;

            case 3:
                if(timerCase3.milliseconds()>patru)
                {
                    pozIntake=pozIntakeInchis;
                    SWITCHSUS++;
                    timerCase4.reset();
                    break;
                }
            case 4:
                pozE=EBlocat;
                pozServoExtindor=pozServoExtindor1;
                if(timerCase4.milliseconds()>cleste){
                    Intake.setPosition(pozIntakeDeschis);
                    SWITCHSUS++;
                    timerBrat.reset();
                }

                break;
            case 5:
                directieMotorBratSus=true;
                if(timerBrat.milliseconds()>cinci) {
                    pozBrat = pozBratSus;
                    DejaSus=true;
                    PaharJos=false;
                    SWITCHSUS=1;
                }
                break;
        }
    }
    public void autoSegJos(){
        pozBrat = pozBratJos;
        if (!deMaiMulte)
        {
            timerJos.reset();
            deMaiMulte=true;
        }
        if (timerJos.milliseconds() > sase) {
            directieMotorBratSus = false;
            pozPuta=pozPutaFara;
            PaharSus = false;
            DejaSus = false;
            deMaiMulte=false;
        }
    }
    public double toCm(double value){
        return value*2.54;
    }
}
//<3<3<3<3<3<3<3<3<3<3<3<3<3<3muie<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3
