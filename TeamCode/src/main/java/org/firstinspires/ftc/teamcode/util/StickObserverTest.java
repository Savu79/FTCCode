/*package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CVMaster;

@TeleOp(name = "StickObserverTest", group="test")

public class StickObserverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        CVMaster cv = new CVMaster(this);
//      call the function to startStreaming
        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.b) break;
        }
        cv.stopCamera();
    }
}*/

