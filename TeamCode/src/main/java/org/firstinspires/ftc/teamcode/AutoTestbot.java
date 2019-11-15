package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AUTO-DRIVE", group="testbot")
//@Disabled
public class AutoTestbot extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware
    double PIDpow = 0;

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
        //press that start button
        waitForStart();
        runtime.reset();
        //run loop while button pressed
        while (opModeIsActive() && runtime.seconds() < 29) {
            //phase 1 forward to grab
            timeMove(0.0,0.2,1300);
            sleep(2000);
            robot.bar.setPosition(0);
            sleep(2000);
            timeMove(0.0,-0.2,1500);
            sleep(2000);
            robot.bar.setPosition(0.5);
            sleep(2000);
            timeMove(-0.2,0.0,1500);
            robot.move2D(0,0,0);
        }
    }
    private void teleUpdate(){
        telemetry.addData("Robot Error = " , gyro.angle_error);
        telemetry.addData("Robot Heading = " , gyro.getAngle());
        telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
        telemetry.update();
    }
    private void timeMove(double forw, double side, int mTime){
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < mTime)) {
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();
            double funcF = -Math.pow(mTime/2,2)*Math.pow((runtime.milliseconds()-(mTime/2)),2)+1;
            double funcS = -Math.pow(mTime/2,2)*Math.pow((runtime.milliseconds()-(mTime/2)),2)+1;
            robot.move2D(forw*funcF,side*funcS,0);
        }
        robot.move2D(0,0,0);
    }
}

