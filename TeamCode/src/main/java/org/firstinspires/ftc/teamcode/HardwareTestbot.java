package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareTestbot
{
    //define opmode members

    private LinearOpMode myOpMode;
    //access instruments of Hub
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    //sensors
    ColorSensor cSensor;
    // motor declarations
    public DcMotor FLeft   = null;
    public DcMotor FRight  = null;
    public DcMotor RLeft   = null;
    public DcMotor RRight  = null;
    // motor for lift
    public Servo   grab    = null;
    public Servo   bar     = null;
    public DcMotor lift     = null;

    //motor powers
    public double MAX_POWER = 0.8;

    public HardwareTestbot(){
    }

    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        cSensor = myOpMode.hardwareMap.colorSensor.get("color");

        FLeft = myOpMode.hardwareMap.get(DcMotor.class, "FLeft");
        FRight = myOpMode.hardwareMap.get(DcMotor.class, "FRight");
        RLeft = myOpMode.hardwareMap.get(DcMotor.class, "RLeft");
        RRight = myOpMode.hardwareMap.get(DcMotor.class, "RRight");

        lift = myOpMode.hardwareMap.get(DcMotor.class,"lift");
        grab  = myOpMode.hardwareMap.get(Servo.class, "grab");
        bar = myOpMode.hardwareMap.get(Servo.class, "bar");

        grab.setPosition(0.25);
        bar.setPosition(0.5);

        //encoderState("off");
        //encoderState("run");
        //encoderState("reset");
        //Brakes the Motors
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLeft.setDirection(DcMotor.Direction.REVERSE);
        FRight.setDirection(DcMotor.Direction.REVERSE);
        RLeft.setDirection(DcMotor.Direction.REVERSE);
        RRight.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setPower(0);
        FLeft.setPower(0);
        FRight.setPower(0);
        RLeft.setPower(0);
        RRight.setPower(0);
        //lift.setZeroPowerBehavior;
    }
    public void move2D(double forw, double side, double spin) {
        double FLPow = -forw + side + spin;
        double FRPow = forw + side + spin;
        double RLPow = -forw - side + spin;
        double RRPow = forw - side + spin;
        // normalize all motor speeds so no values exceeds 100%.
        FLPow = Range.clip(FLPow, -MAX_POWER, MAX_POWER);
        FRPow = Range.clip(FRPow, -MAX_POWER, MAX_POWER);
        RLPow = Range.clip(RLPow, -MAX_POWER, MAX_POWER);
        RRPow = Range.clip(RRPow, -MAX_POWER, MAX_POWER);
        // Set drive motor power levels.
        FLeft.setPower(FLPow);
        FRight.setPower(FRPow);
        RLeft.setPower(RLPow);
        RRight.setPower(RRPow);
    }
    public void encoderState(String a){
        if(a.equals("reset")){
            FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if( a.equals("run")){
            FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(a.equals("position")){
            FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else if(a.equals("off")){
            FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}