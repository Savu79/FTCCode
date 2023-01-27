package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@TeleOp (name="gamepadTest", group = "drive")
public class gamepadTest extends LinearOpMode {
    public void runOpMode()
    {
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("left_trigger ", gamepad1.left_trigger);
            telemetry.addData("left_stick_x ", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.left_stick_y ",gamepad1.left_stick_y);
            telemetry.addData("gamepad1.left_bumper ", gamepad1.left_bumper);
            telemetry.addData("gamepad1.left_stick_button ",gamepad1.left_stick_button);
            telemetry.addData("right_trigger ", gamepad1.right_trigger);
            telemetry.addData("right_stick_x ", gamepad1.right_stick_x);
            telemetry.addData("gamepad1.right_stick_y ",gamepad1.right_stick_y);
            telemetry.addData("gamepad1.right_bumper ", gamepad1.right_bumper);
            telemetry.addData("gamepad1.right_stick_button ",gamepad1.right_stick_button);
            telemetry.update();
        }
    }

}
