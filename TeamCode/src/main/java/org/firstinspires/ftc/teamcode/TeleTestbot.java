package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DRIVE-MODE", group="testbot")
//@Disabled
public class TeleTestbot extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware
    //jazz
    private boolean dollarFound;      // Sound file present flags
    private boolean sevenFound;
    private int dollarmenuID;
    private int sevensixteythreeID;
    private boolean wasA = false;   // Gamepad button history variables
    private boolean WasB = false;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        initJazz();
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) { sleep(5); idle(); }
        telemetry.addData("imu calib status: ", robot.imu.getCalibrationStatus().toString());
        telemetry.update();
        //-------------------- CHANGEABLE----------------------------------
        gyro.target_Angle = 0;
        //-----------------------------------------------------------------
        telemetry.addData(">", "Target Angle Set To: " + gyro.target_Angle);
        telemetry.update();
        //confirm
        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        while (!isStarted()) { telemetry.addData(">", "Robot Heading = ", gyro.getAngle());telemetry.update(); }
        //press that start button
        waitForStart();
        //run loop while button pressed
        while (isStarted()) {
            playJazz(gamepad1.a, gamepad1.b);
            move2D(gamepad1.left_stick_y, gamepad1.left_stick_x);
            robot.armDrive.setPower(gamepad1.right_stick_y/2);
            telemetry.addLine("Robot Error = " + gyro.angle_error);
            telemetry.addLine("Robot Heading = " + gyro.getAngle());
            telemetry.addLine("Robot PID Correction = " + gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
            telemetry.update();
        }
    }
    //movement along 2d and rotation
    private void move2D(double forw, double spin) {
        double LPow = forw + spin;
        double RPow = forw - spin;
        robot.leftDrive.setPower(Range.clip(LPow, -0.90, 0.90));
        robot.rightDrive.setPower(Range.clip(RPow, -0.90, 0.90));
    }
    //initalizes sound play back
    private void initJazz(){
        dollarmenuID = hardwareMap.appContext.getResources().getIdentifier("dollarmenu", "raw", hardwareMap.appContext.getPackageName());
        sevensixteythreeID  = hardwareMap.appContext.getResources().getIdentifier("sevensixteythree",   "raw", hardwareMap.appContext.getPackageName());
        if (dollarmenuID != 0) { dollarFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, dollarmenuID); }
        if (sevensixteythreeID != 0) { sevenFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, sevensixteythreeID); }
        telemetry.addData("dollar sound: ",   dollarFound ?   "Found" : "NOT found\n Add dollarmenu.mp3 to /src/main/res/raw" );
        telemetry.addData("seven sound: ", sevenFound ? "Found" : "Not found\n Add sevensixteythree.mp3 to /src/main/res/raw" );
    }
    //plays sounds
    private void playJazz(boolean pressA, boolean pressB){
        if (dollarFound && pressA && !wasA) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext,dollarmenuID );
            telemetry.addData("Playing", "Resource Silver");
            telemetry.update();
        }
        if (sevenFound && pressB && !WasB) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, sevensixteythreeID);
            telemetry.addData("Playing", "Resource Gold");
            telemetry.update();
        }
        wasA = pressA;
        WasB = pressB;
    }
}

