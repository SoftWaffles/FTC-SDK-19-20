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
        telemetry.addData(">", "Target Angle Set");
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
            if(PIDorRegr(gamepad1.a,gamepad1.b)){
                PID_power = gyro.calcAngle(0);
                move2D(0,0,PID_power);
            }else{
                //simple cube root function to get motor power curve expoFac is just holding variable for power
                double expoFac = gyro.calcAngle(0);
                if(Math.abs(gyro.angle_error) > 8){
                    expoFac = Math.cbrt((5000*(Math.abs(gyro.angle_error)-45))) + 70;
                    expoFac = (180 - gyro.angle_error/Math.abs(180 - gyro.angle_error))*(expoFac/160);
                    move2D(0,0,Range.clip(PID_power,-0.9,0.9));
                }
            }
            telemetry.addLine("Robot Error = %d" + gyro.angle_error);
            telemetry.addLine("Robot Heading = %d" + gyro.getAngle());
            telemetry.update();
        }
    }
    //set the respective motion control math
    public boolean PIDorRegr(boolean a, boolean b){
        if(a = true){
            PIDOn = true;
            telemetry.addLine("Robot Correction Set to PID" );
            telemetry.update();
        }else if(b = true){
            PIDOn = false;
            telemetry.addLine("Robot Correction Set to Regression" );
            telemetry.update();
        }
        return PIDOn;
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

