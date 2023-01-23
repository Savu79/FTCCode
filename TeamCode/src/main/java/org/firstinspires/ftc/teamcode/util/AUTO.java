package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDriveCancelable;

@Autonomous(name="Auto", group = "autonom")
public class AUTO extends LinearOpMode {
    Servo ServoSt;
    Servo ServoDr;
    Servo Intake;
    DcMotorEx Brat;
    DcMotorEx Spinner;

    boolean x=false, y=false;
    int vit=1;

    //Servo St&Dr
    double JosSt=0.04;
    double JosDr=0.96;
    double SusSt=0.57;
    double SusDr=0.43;
    double pozSt = SusSt;
    double pozDr = SusDr;
    double kx=20;

    //Brat
    int SBrat=2000;
    int JBrat=0;
    int pozBrat=JBrat;
    int ky=50;
    double pwrBrat=0.5;

    //Intake
    double DIntake=0;
    double IIntake=0.17;
    double pozIntake=DIntake;

    //Spinner
    int StartSpinner=0;
    int FinishSpinner=2900;
    double pwrSpinner=0.5;
    int kS=30;
    boolean St=false;
    boolean Dr=false;

    public void runOpMode() throws InterruptedException{

        ServoDr = hardwareMap.get(Servo.class,"ServoDr");
        ServoSt = hardwareMap.get(Servo.class,"ServoSt");
        Intake  = hardwareMap.get(Servo.class, "Intake");
        Brat    = hardwareMap.get(DcMotorEx.class, "Brat");
        Spinner = hardwareMap.get(DcMotorEx.class, "Spinner");

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Intake.setPosition(DIntake);
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        waitForStart();
        ServoSt.setPosition(pozSt);
        ServoDr.setPosition(pozDr);
        sleep(200);
        drive.setWeightedDrivePower(
                new Pose2d(
                        -0.8,
                        0,
                        0
                )
        );
        sleep(650);
        drive.setWeightedDrivePower(
                new Pose2d(
                        0,
                        0,
                        0
                )

        );
        sleep(200);
        ServoSt.setPosition(JosSt);
        ServoDr.setPosition(JosDr);
        sleep(200);
        /*
        ServoDr.setPosition(SusDr);
        ServoSt.setPosition(SusSt);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(60) // pune mai mult sau mai putin daca trebe
                .build();

        drive.followTrajectoryAsync(traj1);

        ServoSt.setPosition(JosSt);
        ServoDr.setPosition(JosDr);
        sleep(300);
*/
    }
}