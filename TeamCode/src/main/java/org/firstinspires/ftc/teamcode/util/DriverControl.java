package org.firstinspires.ftc.teamcode.util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp (name="DriverControl", group = "drive")
public class DriverControl extends LinearOpMode {


    boolean x=false, y=false;
    int vit=1;

    //Servo St&Dr
    double JosSt=0.04;
    double JosDr=0.96;
    double SusSt=0.57;
    double SusDr=0.43;
    double pozSt = JosSt;
    double pozDr = JosDr;
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

    int pozSpinner=StartSpinner;

    Servo ServoSt;
    Servo ServoDr;
    Servo Intake;
    DcMotorEx Brat;
    DcMotorEx Spinner;

    ElapsedTime timerY1 = new ElapsedTime();
    ElapsedTime timerX2 = new ElapsedTime();

    public void runOpMode() throws InterruptedException {


        ServoDr = hardwareMap.get(Servo.class,"ServoDr");
        ServoSt = hardwareMap.get(Servo.class,"ServoSt");
        Intake  = hardwareMap.get(Servo.class, "Intake");
        Brat    = hardwareMap.get(DcMotorEx.class, "Brat");
        Spinner = hardwareMap.get(DcMotorEx.class, "Spinner");

        Spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);

        Brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Brat.setTargetPosition(JBrat);
        Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        Brat.setPower(pwrBrat);

        Spinner.setTargetPosition(StartSpinner);
        Spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Spinner.setPower(pwrSpinner);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Viteze
            if(gamepad1.dpad_left)  vit=3;
            if(gamepad1.dpad_down)  vit=2;
            if(gamepad1.dpad_right) vit=1;

            // Condus
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y/vit,
                            gamepad1.right_stick_x/vit,
                            -gamepad1.left_stick_x/vit
                    )
            );
            drive.update();
            telemetry.addData("update","update");
            telemetry.update();
            // Motoare/Servouri
            ServoSt.setPosition(pozSt);
            ServoDr.setPosition(pozDr);
            Intake.setPosition(pozIntake);
            Brat.setTargetPosition(pozBrat);
            Spinner.setTargetPosition(pozSpinner);


            Brat.setPower(pwrBrat);
            Spinner.setPower(pwrSpinner);

            // ServoSt & ServoDr
            if (gamepad2.x && timerX2.milliseconds()>300) {
                if(x)
                {
                    pozSt = SusSt;
                    pozDr = SusDr;
                }
                else
                {
                    pozSt=JosSt;
                    pozDr=JosDr;
                }
                x= !x;
                timerX2.reset();
            }
            if(gamepad2.left_stick_y!=0 && (pozSt>=JosSt||pozSt<=SusSt) && (pozDr<=JosDr||pozDr>=SusDr)) {
                pozSt -= (double)gamepad2.left_stick_y/kx;
                pozDr += (double)gamepad2.left_stick_y/kx;
            }
            if(pozSt<JosSt) pozSt=JosSt;
            if(pozSt>SusSt)   pozSt=SusSt;
            if(pozDr>JosDr) pozDr=JosDr;
            if(pozDr<SusDr)   pozDr=SusDr;

            //Brat
            if(gamepad1.y && timerY1.milliseconds()>300) {
                if(y)
                    pozBrat=SBrat;
                else
                    pozBrat=JBrat;
                y= !y;
                timerY1.reset();
            }
            if(gamepad1.right_trigger!=0 && pozBrat>=(JBrat-10) && pozBrat<=(SBrat+10)) {
                pozBrat+=(int)gamepad1.right_trigger*ky;

            }
            if(gamepad1.left_trigger!=0 && pozBrat>=(JBrat-10) && pozBrat<=(SBrat+10)){
                pozBrat-=(int)gamepad1.left_trigger*ky;
            }
            if(pozBrat<JBrat) pozBrat=JBrat;
            if(pozBrat>SBrat) pozBrat=SBrat;
            if(gamepad2.right_bumper)
                pozBrat=200;


            //Intake
            if(gamepad2.a)
                pozIntake=IIntake;
            if(gamepad2.b)
                pozIntake=DIntake;

            //Spinner
            if(gamepad2.right_stick_x!=0 && pozSpinner>=(StartSpinner-10) && pozSpinner<=(FinishSpinner+10) && ServoSt.getPosition()>0.06)
            {
                //if(gamepad2.right_stick_x<0)
                pozSpinner+=(int)gamepad2.right_stick_x*kS;
            }
            if(pozSpinner<StartSpinner)
                pozSpinner=StartSpinner;
            if(pozSpinner>FinishSpinner)
                pozSpinner=FinishSpinner;

            telemetry.addData("ServoSt: ", ServoSt.getPosition());
            telemetry.addData("ServoDr: ", ServoDr.getPosition());
            telemetry.addData("Brat: ", Brat.getCurrentPosition());
            if(Intake.getPosition()<0.1)
                telemetry.addLine("Intake Deschis");
            else telemetry.addLine("Intake Inchis");
            telemetry.addData("Spinner: ", Spinner.getCurrentPosition());
            telemetry.update();
        }

    }

}
//<3<3<3<3<3<3<3<3<3<3<3<3<3<3muie<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3
