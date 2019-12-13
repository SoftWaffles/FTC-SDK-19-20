package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AUTO-DRIVE-STONE", group="testbot")
//@Disabled
public class AutoTestbotFoundation extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        // Send telemetry message to alert driver that we are calibrating;
        //telemetry.addData(">", "Calibrating Gyro");    //
        //telemetry.update();
        //while (!isStopRequested() && !robot.imu.isGyroCalibrated()) { sleep(50); idle(); }
        //telemetry.addData("imu calib status: ", robot.imu.getCalibrationStatus().toString());
        //telemetry.update();
        //confirm
        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        //press that start button
        waitForStart();
        runtime.reset();
        //run loop while button pressed
        while (opModeIsActive() && runtime.seconds() < 29) {
            gyro.gyroDrive(-0.4,0.0,0,4);
            gyro.gyroDrive(0.4,0.0,0,2);
            robot.bar.setPosition(1);
            gyro.gyroDrive(0,0,270,4);
            gyro.gyroDrive(-0.4, 0,0,1);
            robot.bar.setPosition(0.3);
        }
    }
    private void teleUpdate(){
        telemetry.addData("Robot Error = " , gyro.angle_error);
        telemetry.addData("Robot Heading = " , gyro.getAngle());
        telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
        telemetry.update();
    }
}

