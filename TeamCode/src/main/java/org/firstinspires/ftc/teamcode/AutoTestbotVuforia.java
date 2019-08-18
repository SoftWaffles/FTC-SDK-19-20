package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Math.abs;

@Autonomous(name="TestofVuforia", group="testbot")
@Disabled
public class AutoTestbotVuforia extends LinearOpMode {

    /* Declare OpMode members. */
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    private double PID_power;
    private int prevTarget;
    //vuforia power
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
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
        if(abs(target - prevTarget)> 15){
            PID_power = gyro.calcAngle(target);
            robot.leftDrive.setPower(-PID_power);
            robot.rightDrive.setPower(PID_power);
        }
        prevTarget = target;
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

