package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="test motor", group="test")
public class TestMotor extends LinearOpMode {
    DcMotorEx motor;
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        motor=hardwareMap.get(DcMotorEx.class, "MotorBrat");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Pozitie ", motor.getCurrentPosition());
            telemetry.addData("Power", gamepad1.right_stick_y);
            telemetry.update();
            if(gamepad1.right_stick_button)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setPower(gamepad1.right_stick_y);
        }
    }
}
