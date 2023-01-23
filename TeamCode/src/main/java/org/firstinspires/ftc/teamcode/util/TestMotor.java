package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

@TeleOp(name="test motor", group="drive")
public class TestMotor extends LinearOpMode {
    public void runOpMode(){
        DcMotorEx motor;
        Encoder ff;
        motor=hardwareMap.get(DcMotorEx.class, "motor");
        ff= new Encoder(hardwareMap.get(DcMotorEx.class, "motor"));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            int x=ff.getCurrentPosition();

            telemetry.addData("Pozitie ", x);
            telemetry.addData("Pozitie ", ff.getDirection());
            telemetry.update();
            if(gamepad1.right_stick_button)
            {
                motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }
            motor.setPower(gamepad1.right_stick_y);
        }
    }
}
