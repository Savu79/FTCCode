package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntakeServoExtindor", group = "test")
public class Intake_ServoExtindor_Extindor extends LinearOpMode {

    Servo Intake;
    Servo ServoExtindor;
    DcMotorEx ExtindorDr;
    DcMotorEx ExtindorSt;
    double pozIntake=0;
    double pozServoExtindor=0;

    public static int EInchis = 0;
    public static int EDeschis = 2700;
    int pozE = EInchis;
    public static double powerE = 1;
    public static double kx = 20;

    public void runOpMode(){
        waitForStart();

        Intake=hardwareMap.get(Servo.class, "Intake");
        ServoExtindor=hardwareMap.get(Servo.class, "ServoExtindor");
        ExtindorDr = hardwareMap.get(DcMotorEx.class, "ExtindorDr");
        ExtindorSt = hardwareMap.get(DcMotorEx.class, "ExtindorSt");

        ExtindorSt.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtindorDr.setDirection(DcMotorSimple.Direction.REVERSE);

        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtindorDr.setTargetPosition(EInchis);
        ExtindorSt.setTargetPosition(EInchis);
        ExtindorSt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtindorDr.setPower(powerE);
        ExtindorSt.setPower(powerE);

        while (opModeIsActive()) {
            ExtindorDr.setTargetPosition(pozE);
            ExtindorSt.setTargetPosition(pozE);
            Intake.setPosition(pozIntake);
            ServoExtindor.setPosition(pozServoExtindor);
            telemetry.addData("PozitieIntake ", Intake.getPosition());
            telemetry.addData("PozitieServoExtindor", ServoExtindor.getPosition());
            telemetry.addData("PozitieExtindor", ExtindorDr.getCurrentPosition());
            telemetry.update();

            if (gamepad2.left_stick_y!=0 && pozE<=EDeschis && pozE>=EInchis)
                pozE-=gamepad2.left_stick_y*kx;
            if (pozE<EInchis)
                pozE=EInchis;
            if (pozE>EDeschis)
                pozE=EDeschis;
            if (gamepad1.right_stick_y != 0 && pozIntake >= 0 && pozIntake <= 1) {
                pozIntake -= gamepad1.right_stick_y / 100;
            }
            if(pozIntake<0)
                pozIntake=0;
            if(pozIntake>1)
                pozIntake=1;
            if (gamepad1.left_stick_y != 0 && pozServoExtindor >= 0 && pozServoExtindor <= 1) {
                pozServoExtindor -= gamepad1.left_stick_y / 100;
            }
            if(pozServoExtindor<0)
                pozServoExtindor=0;
            if(pozServoExtindor>1)
                pozServoExtindor=1;
        }
    }
}
