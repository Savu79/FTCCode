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

    //Brat
    public static int BInchis=10;
    public static int BDeschis=2200;

    public static double a=0;

    public static double Kp=0.007;
    public static double Ki=0.002;
    public static double Kd=0;
    public static int kB=30;

    int reference=BInchis;
    int lastReference=reference;

    int error;
    int lastError;

    double integralSum=0;
    public static double integralSumLim=3;

    double derivative;
    float currentFilterEstimate;
    float previousFilterEstimate;

    int pozB;

    ElapsedTime timer = new ElapsedTime();


    //Extindor
        public static int EInchis = 0;
        public static int EDeschis = 2700;
        int pozE = EInchis;
        public static double powerE = 1;
        public static int kE = 20;


    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    DcMotorEx Brat;

    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

           Brat    = hardwareMap.get(DcMotorEx.class, "Brat");
        ExtindorDr = hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt = hardwareMap.get(DcMotorEx.class, "ExtindorSt");

              Brat.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);

        Brat.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtindorSt.setTargetPosition(EInchis);
        ExtindorSt.setTargetPosition(EInchis);
        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setPower(powerE);
        ExtindorSt.setPower(powerE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Viteze - gamepad1 ///////////////////////////////////////////////////////////////////
            if(gamepad1.dpad_left)  vit=3;
            if(gamepad1.dpad_down)  vit=2;
            if(gamepad1.dpad_right) vit=1;

            // Condus  - gamepad1 //////////////////////////////////////////////////////////////////
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y/vit,
                            -gamepad1.right_stick_x/vit,
                            -gamepad1.left_stick_x/vit
                    )
            );
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            //Brat   - gamepad2 ////////////////////////////////////////////////////////////////////
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

            if(gamepad2.x) pozB=BInchis;
            if(gamepad2.y) pozB=BDeschis;

            if(gamepad2.right_stick_y!=0 && pozB<=BDeschis && pozB>=BInchis)
                pozB-=(int)(-gamepad2.right_stick_y*kB);
            if(pozB>BDeschis)
                pozB=BDeschis;
            if(pozB<BInchis)
                pozB=BInchis;


            //Extindor -- gamepad2 /////////////////////////////////////////////////////////////////
            ExtindorDr.setTargetPosition(pozE);
            ExtindorSt.setTargetPosition(pozE);

            if(gamepad2.b) pozE=EDeschis;
            if(gamepad2.a) pozE=EInchis;

            if (gamepad2.left_stick_y!=0 && pozE<=EDeschis && pozE>=EInchis)
                pozE-=gamepad2.left_stick_y*kE;
            if (pozE<EInchis)
                pozE=EInchis;
            if (pozE>EDeschis)
                pozE=EDeschis;


            //Puteri ///////////////////////////////////////////////////////////////////////////////
            Brat.setPower((error*Kp) + (integralSum*Ki) + (derivative*Kd));
            ExtindorSt.setPower(powerE);
            ExtindorDr.setPower(powerE);


            //reset  - gamepad2/////////////////////////////////////////////////////////////////////
            if (gamepad2.right_stick_button)
                Brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(gamepad2.left_stick_button)
            {
                ExtindorDr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ExtindorSt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //Telemetry ////////////////////////////////////////////////////////////////////////////
            telemetry.addData("Pozitie robot X: ", poseEstimate.getX());
            telemetry.addData("Pozitie robot Y: ", poseEstimate.getY());
            telemetry.addData("Heading robot: ", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Brat: ", Brat.getCurrentPosition());
            telemetry.addData("Extindor: ", ExtindorDr.getCurrentPosition());
            telemetry.update();
        }

    }

}
//<3<3<3<3<3<3<3<3<3<3<3<3<3<3muie<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3<3
