package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Math.abs;

@Autonomous(name="TEST: Vuforia", group="testbot")
//@Disabled
public class AutoTestbotVuforia extends LinearOpMode {

    /* Declare OpMode members. */
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    private double PID_power;
    private int prevTarget;
    //vuforia power
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AZjoEnH/////AAABmXdFD2Xsrk4krInf+EdRY0NaRrdzvbptLaUoVN2kuF2/FnuWVscRF9ozak4bIpJCr1SLehfzrXHS+H3Z7XMNIgxwg6lttQ4zp7ODEDt1XQ/DLQcjpmYXruF4eBBRsIBey35Ue6g4E51WOebmNW/aDFDhz3zON+NNYbyk/4XOszsw7CwHpcNLBXqT0prM/NYwkCaJFocA8cpWcViM0Mka8kEV+T1X1ZtRnPwMxtQrxO19ksdbRv0bjPmco0iiOAvRwMcyVxg250tckD64iSWJkIhlqakYMLA1r00YPtUY4VSfShG0pWTDn/RF9/TqhM8qICp9ZPCz5QlPn8qt4cfiofTjzE41R+VvjKnIGK1B9g5o";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) { initTfod(); }
        else { telemetry.addData("Sorry!", "This device is not compatible with TFOD"); }
        while (!isStarted()) { telemetry.addData(">", "Robot Heading = %d", gyro.getAngle());telemetry.update(); }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }
        if (tfod != null) { tfod.shutdown(); }
    }
    private void move2D(double forw, double side, double spin){
        double LPow = forw + spin;
        double RPow = forw - spin;
        robot.leftDrive.setPower(Range.clip(LPow, -0.90,0.90));
        robot.rightDrive.setPower(Range.clip(RPow, -0.90,0.90));
    }
    //just vuforia engine starting
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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

