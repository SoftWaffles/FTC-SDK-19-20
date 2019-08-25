package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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
            move2D(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addLine("Robot Mid Translation = " + midTranslation() + " || " + robot.TOTAL_MOTOR_POS);
            telemetry.addLine("Robot Error = " + gyro.angle_error);
            telemetry.addLine("Robot Heading = " + gyro.getAngle());
            telemetry.addLine("Robot PID Correction = " + gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
            telemetry.update();
        }
    }
    public double midTranslation(){
        double mid_Trans = 0;
        robot.TOTAL_MOTOR_POS += robot.midDrive.getCurrentPosition() - robot.PREV_MOTOR_POS;
        robot.PREV_MOTOR_POS = robot.TOTAL_MOTOR_POS;
        mid_Trans = robot.TOTAL_MOTOR_POS/robot.COUNTS_PER_INCH;
        return mid_Trans;
    }
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

