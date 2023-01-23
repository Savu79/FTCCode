package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="TFOD", group="drive")
public class TFOD extends LinearOpMode {
    int i=0;
    private static final String[] TFOD_MODEL_ASSET = {
            "model_TUDORC.tflite",
            "SoareCOMPLET.tflite"
    };
    private static final String[] LABELS = {
            "TudorC",
            "Soare"
    };
    private ElapsedTime CRuntime = new ElapsedTime();
    private ElapsedTime durataMeci = new ElapsedTime();
    private static final String VUFORIA_KEY =
            "AQXzaeH/////AAABma+dmoRw1kZHgWbfDr88vn4I2y/JjiEnuuQvCZjhNbwWZE1CdaCGcKPWc5Pot143CxXBXDQyqZMQTyDqbBXzxe5YovSPVlPpa0LiIecLfVPYVnWkqng1tm8B1RNaeyYx25fkGI/LAu3Qeq/KlVPHN+iUClyDowkjXejs8+wzBp6wdzUEjzwEcnrkruqirmZEDtwnPKKHHOItKj9n7TzS77RtS6fSF0P9Qtlfi58Lg3kT3v/6ml7slcQMCGIWbBZRA9EYatWvK6ffM/TL9Xvr0jqaAiho0fQiilxGg2GvMRzc3qFyqA4TwPLrvn41C620ufoVvPAgmk7+at56tpE7/Y9hugGE1ML8jZRdqx9iqC6r";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.update();
                    }
                }
                if(gamepad1.b) {
                    assert tfod != null;
                    tfod.deactivate();
                    i++;
                    break;
                }
            }
            while(opModeIsActive())
            {
                initTfod();
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.update();
                    }
                }
            }

        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET[i], LABELS );

    }
}