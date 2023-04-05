package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@TeleOp (name= "Drive", group = "test")
public class Drive extends LinearOpMode {

    double vit=1;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.dpad_left) vit=3;
            if(gamepad1.dpad_up) vit=2;
            if(gamepad1.dpad_right) vit=1;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y/vit,
                            -gamepad1.right_stick_x/vit,
                            -gamepad1.left_stick_x/vit
                    )
            );
            drive.update();
            Pose2d poseEstimate=drive.getPoseEstimate();
        }
    }
}
