package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="TestOfPID", group="testbot")
//@Disabled
public class AutoTestbot extends LinearOpMode {

    /* Declare OpMode members. */
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    double PID_power;
    boolean PIDOn;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) { sleep(50); idle(); }
        telemetry.addData("imu calib status: ", robot.imu.getCalibrationStatus().toString());
        telemetry.update();
        //confirm
        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        while (!isStarted()) { telemetry.addData(">", "Robot Heading = %d", gyro.getAngle());telemetry.update(); }
        //press that start button
        waitForStart();
        //run loop while button pressed
        while (isStarted()) {
            //CONVENTIONS USED COUNTERCLOCKWISE IS NEGATIVE TURN ----- CLOCKWISE IS POSITIVE TURN
            PID_power = gyro.calcAngle(0);
            move2D(0,0,PID_power);

            telemetry.addData("Robot Error = " , gyro.angle_error);
            telemetry.addData("Robot Heading = " , gyro.getAngle());
            telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
            telemetry.update();
        }
    }
    //method for movement
    public void move2D(double forw, double side, double spin){
        double LPow = forw + spin;
        double RPow = forw - spin;
        robot.leftDrive.setPower(Range.clip(LPow, -0.90,0.90));
        robot.rightDrive.setPower(Range.clip(RPow, -0.90,0.90));
        if(Math.abs(side) >= 0.3){
            robot.midDrive.setPower(Range.clip(side, -0.90,0.90));
        }
    }
}

