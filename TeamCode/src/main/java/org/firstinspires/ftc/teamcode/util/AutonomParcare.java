package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.PowerPlayDeterminationExample.parcare;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="AutonomParcare")
@Config
public class AutonomParcare extends LinearOpMode {
    Servo Intake;
    Servo ServoExtindor;
    Servo Puta;
    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    DcMotorEx Brat;
    DcMotorEx MotorBrat;

    static double pozIntakeDeschis = 0.17;
    static double pozIntakeInchis = 0;
    public static double pozIntakePasare = 0.1;
    double pozIntake;

    static double pozServoExtindorMij = 0.46;
    static double pozServoExtindor1 = 0.06;
    static double pozServoExtindor2 = 0.10;
    static double pozServoExtindor3 = 0.15;
    static double pozServoExtindor4 = 0.21;
    static double pozServoExtindor5 = 0.25;
    public static double pozServoExtindorPas = 0.67;
    static double pozServoExtindorJos = 0;
    static double pozServoExtindor = pozServoExtindor1;

    double pozPutaCu=0.87;
    double pozPutaFara=1;
    double pozPuta=pozPutaFara;

    int poz;
    private PowerPlayDeterminationExample sleeveDetection;
    private OpenCvCamera phoneCam;
    public void runOpMode(){
        Intake       =hardwareMap.get(Servo.class, "Intake");
        ServoExtindor=hardwareMap.get(Servo.class, "ServoExtindor");
        ExtindorDr   =hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt   =hardwareMap.get(DcMotorEx.class, "ExtindorSt");
        Puta         =hardwareMap.get(Servo.class, "Puta");
        Brat         =hardwareMap.get(DcMotorEx.class, "Brat");
        MotorBrat    =hardwareMap.get(DcMotorEx.class, "MotorBrat");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        sleeveDetection = new PowerPlayDeterminationExample();
        phoneCam.setPipeline(sleeveDetection);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("Pozitie", parcare);
            telemetry.update();
            poz=parcare;
            sleep(200);
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory fata= drive.trajectoryBuilder(new Pose2d(0,0,0)).forward(toInch(54)).build();
        Trajectory stanga= drive.trajectoryBuilder(new Pose2d(0,0,0)).strafeLeft(toInch(65)).build();
        Trajectory dreapta= drive.trajectoryBuilder(new Pose2d(0,0,0)).strafeLeft(toInch(-65)).build();

        waitForStart();

        Puta.setPosition(pozPutaCu);

        ServoExtindor.setPosition(0);
        sleep(1000);
        Intake.setPosition(pozIntakeDeschis);

        drive.followTrajectory(fata);

        ServoExtindor.setPosition(pozServoExtindor5);
        sleep(1000);
        Intake.setPosition(pozIntakeInchis);
        sleep(1000);
        ServoExtindor.setPosition(pozServoExtindorMij);

        if(poz==1)
            drive.followTrajectory(stanga);
        if(poz==3)
            drive.followTrajectory(dreapta);

    }
    public double toCm(double value){
        return value*2.54;
    }
    public double toInch(double value) { return value/2.54; }
}
