package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class HardwareTestbot
{
    //define opmode members

    private LinearOpMode myOpMode;
    private ElapsedTime period  = new ElapsedTime();
    //access instruments of Hub
    BNO055IMU imu;
    Orientation angle;
    Acceleration gravity;
    // motor declarations
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor RLeft = null;
    public DcMotor RRight = null;
    // motor for arm
    public Servo   grab    = null;
    public Servo   spin    = null;
    public Servo   bar     = null;
    public DcMotor Arm     = null;

    //motor powers
    public double             MAX_POWER               = 0.5;
    //motor monitoring
    private static final double     COUNTS_PER_MOTOR_REV    = 1220 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);

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

        FLeft = myOpMode.hardwareMap.get(DcMotor.class, "FLeft");
        FRight = myOpMode.hardwareMap.get(DcMotor.class, "FRight");
        RLeft = myOpMode.hardwareMap.get(DcMotor.class, "RLeft");
        RRight = myOpMode.hardwareMap.get(DcMotor.class, "RRight");

        Arm = myOpMode.hardwareMap.get(DcMotor.class,"arm");
        grab  = myOpMode.hardwareMap.get(Servo.class, "grab");
        spin = myOpMode.hardwareMap.get(Servo.class, "spin");
        bar = myOpMode.hardwareMap.get(Servo.class, "bar");

        grab.setPosition(0.5);
        spin.setPosition(0.64);
        bar.setPosition(1);

        //encoderState("off");
        //encoderState("run");
        //encoderState("reset");
        //Brakes the Motors
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FRight.setDirection(DcMotorSimple.Direction.REVERSE);
        RLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm.setPower(0);
        FLeft.setPower(0);
        FRight.setPower(0);
        RLeft.setPower(0);
        RRight.setPower(0);
        //Arm.setZeroPowerBehavior;
    }
    public void move2D(double forw, double side, double spin) {
        double FLPow = -forw + side + spin;
        double FRPow = forw + side + spin;
        double RLPow = -forw - side + spin;
        double RRPow = forw - side + spin;
        // normalize all motor speeds so no values exceeds 100%.
        FLPow = Range.clip(FLPow, -MAX_POWER/2, MAX_POWER/2);
        FRPow = Range.clip(FRPow, -MAX_POWER/2, MAX_POWER/2);
        RLPow = Range.clip(RLPow, -MAX_POWER/2, MAX_POWER/2);
        RRPow = Range.clip(RRPow, -MAX_POWER/2, MAX_POWER/2);
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

