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
    ElapsedTime runtime = new ElapsedTime();
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    double PID_power;

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
            encoderDrive(0.5,0.5,10,4,2);
            PID_power = gyro.calcAngle(90);
            move2D(0,0,PID_power);
            encoderDrive(0.5,0.5,10,4,2);
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
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup){
        double     COUNTS_PER_MOTOR_REV    = 560 ;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is the ratio between the motor axle and the wheel
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         =  (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((runtime.seconds() < timeoutS) && (Math.abs(robot.leftDrive.getCurrentPosition()) < newLeftTarget  && (Math.abs(robot.rightDrive.getCurrentPosition()) < newRightTarget))){
            double rem = (Math.abs(robot.leftDrive.getCurrentPosition())+Math.abs(robot.rightDrive.getCurrentPosition()))/2;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup){ double ramp = R / rampup; NLspeed = Lspeed * ramp; NRspeed = Rspeed * ramp;}
            //Keep running until you are about two rotations out
            else if(rem > (1000) ) { NLspeed = Lspeed; NRspeed = Rspeed;}
            //start slowing down as you get close to the target
            else if(rem > (200) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {NLspeed = Lspeed * (rem / 1000); NRspeed = Rspeed * (rem / 1000); }
            //minimum speed
            else{ NLspeed = Lspeed * .2; NRspeed = Rspeed * .2; }
            //Pass the seed values to the motors
            robot.leftDrive.setPower(NLspeed);
            robot.rightDrive.setPower(NRspeed);
        }
        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
        double resetC = (Math.abs(robot.leftDrive.getCurrentPosition())+Math.abs(robot.rightDrive.getCurrentPosition()));
        //Get the motor encoder resets in motion
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0){
            resetC =  Math.abs(robot.leftDrive.getCurrentPosition())+Math.abs(robot.rightDrive.getCurrentPosition());
            idle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  sleep(250);   // optional pause after each move
    }
}

