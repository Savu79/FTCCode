package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutonomRosuStanga")
@Config
public class AutonomRosuStanga extends LinearOpMode {

    Servo Intake;
    Servo ServoExtindor;
    Servo Puta;
    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    DcMotorEx Brat;
    DcMotorEx MotorBrat;

    //viteza
    double vit=1;

    boolean schimb;

    //valori timere
    static int zero=1000;
    static int unu=1000;
    static int doi=1400;
    static int trei=1700;
    static int patru=500;
    static int cinci=900;
    static int sase=1500;

    //timer
    ElapsedTime timerBrat= new ElapsedTime();
    ElapsedTime timerCase1 = new ElapsedTime();
    ElapsedTime timerCase2 = new ElapsedTime();
    ElapsedTime timerCase3= new ElapsedTime();
    ElapsedTime timerY = new ElapsedTime();
    ElapsedTime timerJos = new ElapsedTime();

    ////Intake
    double pozIntakeDeschis = 0.17;
    double pozIntakeInchis = 0;
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
    double pozaPutaFara=1;
    double pozPuta=pozaPutaFara;

    //MotorBrat
    boolean directieMotorBratSus=false;
    public static double Kcos=-0.41;
    public static double PowerSin=0.1;
    public static double PowerCos=0.2;
    public static double Ksin=-0.7;

    //Brat
    int pozBratSus = 2200;
    int pozBratJos = 0;
    int pozBrat=pozBratJos;
    public static double powerB=0.6;

    ///Auto
    boolean PaharSus=false;
    boolean DejaSus=false;
    boolean PaharJos=false;
    boolean deMaiMulte=false;
    int SWITCHSUS=1;

    public static double valoare;

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-103/2.54, -165.5/2.54, Math.toRadians(90)))
                .forward(valoare/2.54)
                .build();

        ServoExtindor.setPosition(pozServoExtindor5);

        drive.setPoseEstimate(new Pose2d(-103/2.54, -165.5/2.54, Math.toRadians(90)));
        drive.followTrajectory(traj1);

        while(opModeIsActive()){

            if(timerJos.seconds()>1 && schimb)
            {
                Intake.setPosition(pozIntakeDeschis);
                schimb=!schimb;
                timerJos.reset();
            } else if (timerJos.seconds()>1 && !schimb)
            {
                Intake.setPosition(pozIntakeInchis);
                schimb=!schimb;
                timerJos.reset();
            }
        }
    }
    public double COSSIN(boolean directieMotorBratSus)
    {
        if(directieMotorBratSus)
            return(Math.sin(Math.toRadians(MotorBrat.getCurrentPosition()/(537.7/360))) * Ksin+PowerSin);
        else
            return(Math.cos(Math.toRadians(MotorBrat.getCurrentPosition()/(357.7/360))) * Kcos-PowerCos);
    }
    public void autoSegSus(){
        switch (SWITCHSUS){
            case 1:
                EBlocat=ExtindorDr.getCurrentPosition();
                pozPuta=pozaPutaFara;
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
                    break;
                }
            case 4:
                pozE=EBlocat;
                pozServoExtindor=pozServoExtindor1;
                SWITCHSUS++;
                timerBrat.reset();
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
            pozPuta=pozaPutaFara;
            PaharSus = false;
            DejaSus = false;
            deMaiMulte=false;
        }
    }
    public double toCm(double value){
        return value*2.54;
    }
    public double toInch(double value) { return value/2.54; }
}
