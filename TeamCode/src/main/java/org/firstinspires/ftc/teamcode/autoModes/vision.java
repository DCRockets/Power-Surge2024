package org.firstinspires.ftc.teamcode.autoModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Autonomous(name = "Sample Object Detection")
public class TensorFlowObjectDetection extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "IntoTheDeep.tflite";
    private static final String[] LABELS = {
            "SAMPLE_YELLOW",
            "SAMPLE_RED",
            "SAMPLE_BLUE",
    };

    private static final String VUFORIA_KEY =
            "Af5zogT/////AAABmVLxj0wMJkJfofwMhzBVUbRDklzGSBj633kwMFOwl1xS8J6kgXQZxGqcAojyLNsoysqPHteMORpZK72ej90aQxLfquSeFk2nK5+6QmEqvGfh4kUjIs3t3SB/9W8ElpCayOzFrDoiDL78tWeQRDxk+mzCEbuJh0EPA0FRsNaBDyBizhxZuatkM/MSi3BL7MBND6WG8ccV5y5sn2w6RjPHVjj4Zb9NKfO1zg6AKIOpYjgI7icmrPS+6XMq6MagE+u9Vbv3HTGRL6SsZQUHa1WW/FKr/AyEbZTC9XxZrve0JxWI4n2S/ghp1xTjfhncQhHa9EBFUJOltUvsQeFffb6kqn3B1dAzH7j2sshP3MAW3oIZ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // Set the zoom to be a little closer for TensorFlow (optional).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("Label (%d)", updatedRecognitions.indexOf(recognition)), recognition.getLabel());
                            telemetry.addData(String.format("  Left,Top (%.03f , %.03f)", recognition.getLeft(), recognition.getTop()));
                            telemetry.addData(String.format("  Right,Bottom (%.03f , %.03f)", recognition.getRight(), recognition.getBottom()));
                        }
                        telemetry.update();
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}