package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="TestEncoder", group="test")
public class TestEncoder extends LinearOpMode {
    public void runOpMode (){
        DigitalChannel encoder;
        encoder = (DigitalChannel) hardwareMap.get(AnalogInput.class,"encoder");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("pozitie_encoder",encoder.getState());
            telemetry.update();
        }
    }
}
