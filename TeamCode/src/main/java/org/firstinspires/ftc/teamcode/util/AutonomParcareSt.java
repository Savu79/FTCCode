package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.PowerPlayDeterminationExample.parcare;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="AutonomParcareSt")
@Config
public class AutonomParcareSt extends LinearOpMode {
    private PowerPlayDeterminationExample sleeveDetection;
    private OpenCvCamera phoneCam;
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        sleeveDetection = new PowerPlayDeterminationExample();
        phoneCam.setPipeline(sleeveDetection);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("Pozitie", parcare);
            telemetry.update();
            sleep(200);
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        Trajectory fata= drive.trajectoryBuilder(new Pose2d(0,0,0)).forward(toInch(60)).build();
        Trajectory stanga= drive.trajectoryBuilder(new Pose2d(0,0,0)).strafeLeft(toInch(60)).build();
        Trajectory stanga= drive.trajectoryBuilder(new Pose2d(0,0,0)).strafeLeft(toInch(60)).build();

    }
    public double toCm(double value){
        return value*2.54;
    }
    public double toInch(double value) { return value/2.54; }
}
