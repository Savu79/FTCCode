package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntakeServoExtindor", group = "drive")
public class IntakeServoExtindor extends LinearOpMode {

    Servo Intake;
    Servo ServoExtindor;
    double pozIntake=0;
    double pozServoExtindor=0;
    public void runOpMode(){
        waitForStart();
        Intake=hardwareMap.get(Servo.class, "Intake");
        ServoExtindor=hardwareMap.get(Servo.class, "ServoExtindor");
        while (opModeIsActive()) {
            Intake.setPosition(pozIntake);
            ServoExtindor.setPosition(pozServoExtindor);
            telemetry.addData("PozitieIntake ", Intake.getPosition());
            telemetry.addData("PozitieServoExtindor", ServoExtindor.getPosition());
            telemetry.update();
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
