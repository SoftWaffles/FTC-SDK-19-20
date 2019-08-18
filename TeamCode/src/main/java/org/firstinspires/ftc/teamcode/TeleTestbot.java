package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestofDrive", group="testbot")
//@Disabled
public class TeleTestbot extends LinearOpMode {

    /* Declare OpMode members. */
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
        //-------------------- CHANGEABLE----------------------------------
        gyro.target_Angle = 0;
        //-----------------------------------------------------------------
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
            if(gamepad1.dpad_up){PID_power = gyro.calcAngle(0);}
            else if (gamepad1.dpad_right){PID_power = gyro.calcAngle(90);}
            else if (gamepad1.dpad_down){PID_power = gyro.calcAngle(180);}
            else if (gamepad1.dpad_left){PID_power = gyro.calcAngle(270);}

            robot.leftDrive.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x+PID_power);
            robot.rightDrive.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x+PID_power);
            robot.midDrive.setPower(gamepad1.left_stick_x);

            telemetry.addLine("Robot Mid Translation (inches) = %d" + midTranslation());
            telemetry.addLine("Robot Error = %d" + gyro.angle_error);
            telemetry.addLine("Robot Heading = %d" + gyro.getAngle());
            telemetry.update();
        }
    }
    public double midTranslation(){
        double mid_Trans = 0;
        robot.TOTAL_MOTOR_POS += robot.PREV_MOTOR_POS - robot.midDrive.getCurrentPosition();
        robot.PREV_MOTOR_POS = robot.TOTAL_MOTOR_POS;
        mid_Trans = robot.TOTAL_MOTOR_POS/robot.COUNTS_PER_INCH;
        return mid_Trans;
    }
}

