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

@Autonomous(name="TestofVuforia", group="testbot")
//@Disabled
public class AutoTestbotVuforia extends LinearOpMode {

    /* Declare OpMode members. */
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    private double PID_power;
    private int prevTarget;
    //vuforia power
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String VUFORIA_KEY = "AZjoEnH/////AAABmXdFD2Xsrk4krInf+EdRY0NaRrdzvbptLaUoVN2kuF2/FnuWVscRF9ozak4bIpJCr1SLehfzrXHS+H3Z7XMNIgxwg6lttQ4zp7ODEDt1XQ/DLQcjpmYXruF4eBBRsIBey35Ue6g4E51WOebmNW/aDFDhz3zON+NNYbyk/4XOszsw7CwHpcNLBXqT0prM/NYwkCaJFocA8cpWcViM0Mka8kEV+T1X1ZtRnPwMxtQrxO19ksdbRv0bjPmco0iiOAvRwMcyVxg250tckD64iSWJkIhlqakYMLA1r00YPtUY4VSfShG0pWTDn/RF9/TqhM8qICp9ZPCz5QlPn8qt4cfiofTjzE41R+VvjKnIGK1B9g5o";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        //start up Vuforia Engine
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) { initTfod(); }
        else { telemetry.addData("Sorry!", "This device is not compatible with TFOD"); }
        while (!isStarted()) { telemetry.addData(">", "Robot Heading = %d", gyro.getAngle());telemetry.update(); }
        waitForStart();
        while (isStarted()) {
            telemetry.addLine("Robot Heading = %d" + gyro.getAngle());
            telemetry.update();
            //block target calcs
            if (tfod != null) { tfod.activate();}
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 1) {
                        int goldMineralX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                gyroOrien(visionCalc(goldMineralX));
                            }
                        }
                    }
                }
            }
            if (tfod != null) {tfod.shutdown(); }
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
    private int visionCalc(int objX){
        int target = 0;
        target = (((objX/1080)*(84))-42);
        return target;
    }
    private void gyroOrien(int target){
        double pow = gyro.calcAngle((180+target)%180);
        move2D(0,0,pow);
    }
    private void move2D(double forw, double side, double spin){
        double LPow = forw + spin;
        double RPow = forw - spin;
        robot.leftDrive.setPower(Range.clip(LPow, -0.90,0.90));
        robot.rightDrive.setPower(Range.clip(RPow, -0.90,0.90));
        if(Math.abs(side) >= 0.3){
            robot.midDrive.setPower(Range.clip(side, -0.90,0.90));
        }
    }
    //just vuforia engine starting
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(LABEL_GOLD_MINERAL);
    }
}

