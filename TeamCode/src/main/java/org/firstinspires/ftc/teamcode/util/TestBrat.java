package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Test Brat",group="Test")
public class TestBrat extends LinearOpMode {
    double power=0;
    DcMotorEx Brat;
    public void runOpMode(){
        Brat= hardwareMap.get(DcMotorEx.class, "Brat");
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            Brat.setPower(power);
            telemetry.addData("power",power);
            telemetry.addData("poz",Brat.getCurrentPosition());
            telemetry.update();
            power=-gamepad1.right_stick_y;
            if (gamepad1.right_stick_button){
                Brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
}
