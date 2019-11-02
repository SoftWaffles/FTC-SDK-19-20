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
            gyro.gyroDrive(0,0,3);
            sleep(1000);
            gyro.gyroDrive(0,45,3);
            sleep(1000);
            gyro.gyroDrive(0,90,3);
            sleep(1000);
            gyro.gyroDrive(0,0,0);
            encoderDrive(0.5,20,20,10,1);

            telemetry.addData("Robot Error = " , gyro.angle_error);
            telemetry.addData("Robot Heading = " , gyro.getAngle());
            telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
            telemetry.update();
        }
        move2D(0,0);
    }
    //method for movement
    public void move2D(double forw, double spin){
        double LPow = forw + spin;
        double RPow = forw - spin;
        robot.leftDrive.setPower(Range.clip(LPow, -0.90,0.90));
        robot.rightDrive.setPower(Range.clip(RPow, -0.90,0.90));
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double AccelerationInches, int Direction) {
        // Declares variables that are used for this method
        int NewLeftTarget;
        int NewRightTarget;
        int RightPosition;
        int LeftPosition;
        double LeftPower;
        double RightPower;
        // Resets encoders to 0
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Checks to make sure that encoders are reset.
        while(robot.leftDrive.getCurrentPosition() > 1 && robot.rightDrive.getCurrentPosition()> 1){
            sleep(25);
        }
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            // Calculates the needed encoder ticks by multiplying a pre-determined amount of robot.COUNTS_PER_INCHes,
            // and the method input gets the actual distance travel in inches
            NewLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            NewRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            // Gets the current position of the encoders at the beginning of the EncoderDrive method
            RightPosition = robot.rightDrive.getCurrentPosition();
            LeftPosition = robot.leftDrive.getCurrentPosition();
            // Gives the encoders the target.
            robot.leftDrive.setTargetPosition(NewLeftTarget);
            robot.rightDrive.setTargetPosition(NewRightTarget);

            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(robot.leftDrive.getCurrentPosition() > 1){
                sleep(15);
            }
            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // reset the timeout time and start motion.
            runtime.reset();
            // This gets where the motor encoders will be at full position when it will be at full speed.
            double LeftEncoderPositionAtFullSpeed = ((AccelerationInches*(robot.COUNTS_PER_INCH)) + LeftPosition);
            double RightEncoderPositionAtFullSpeed = ((AccelerationInches*(robot.COUNTS_PER_INCH)) + RightPosition);
            boolean Running = true;
            // This gets the absolute value of the encoder positions at full speed - the current speed, and while it's greater than 0, it will continues increasing the speed.
            // This allows the robot to accelerate over a set number of inches, which reduces wheel slippage and increases overall reliability
            while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && Running && opModeIsActive()) {
                // While encoders are not at position
                if (((Math.abs(speed)) - (Math.abs(robot.leftDrive.getPower()))) > .05){
                    // This allows the robot to accelerate over a set distance, rather than going full speed.  This reduces wheel slippage and increases reliability.
                    LeftPower = (Range.clip(Math.abs((robot.leftDrive.getCurrentPosition()) / (LeftEncoderPositionAtFullSpeed)), .15, speed));
                    RightPower =(Range.clip(Math.abs((robot.rightDrive.getCurrentPosition()) / (RightEncoderPositionAtFullSpeed)), .15, speed));

                    robot.leftDrive.setPower(LeftPower*Direction);
                    robot.rightDrive.setPower(RightPower*Direction);

                    telemetry.addData("Accelerating", RightEncoderPositionAtFullSpeed);
                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                }else if(Math.abs(NewLeftTarget) - Math.abs(robot.leftDrive.getCurrentPosition()) < -1) {
                    Running = false;
                }else{
                    // Multiplies the speed desired by the direction, which has a value of either 1, or -1, and allows for backwards driving with the ramp up function
                    robot.leftDrive.setPower((speed*Direction));
                    robot.rightDrive.setPower((speed*Direction));

                    telemetry.addData("Path1", "Running to %7d :%7d", NewLeftTarget, NewRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                }
                // Display information for the driver.
            }
            // Stops all motion
            // Set to run without encoder, so it's not necessary to declare this every time after the method is used
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set power to 0
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

        }
    }
}

