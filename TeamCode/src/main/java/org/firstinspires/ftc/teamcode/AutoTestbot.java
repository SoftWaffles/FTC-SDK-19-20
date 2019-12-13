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

@Autonomous(name="AUTO-DRIVE-STONE", group="testbot")
//@Disabled
public class AutoTestbot extends LinearOpMode {

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
        while (opModeIsActive() && runtime.seconds() < 29 && !isStopRequested()) {
            gyro.gyroDrive(-0.3,0.0,0,0.7);
            sleep(1000);
            gyro.gyroDrive(0.0,0.3, 0,0.6);
            sleep(1000);
            while(100 < robot.cSensor.alpha() && robot.cSensor.alpha() < 400 && !isStopRequested()){
                robot.move2D(0.0, -0.2, 0);
                teleUpdate();
            }
            gyro.gyroDrive(0,-0.2,0,1);
            gyro.gyroDrive(-0.2,0,0,0.3);
            sleep(1000);
            robot.grab.setPosition(1);
            sleep(500);
            gyro.gyroDrive(0.35,0,0,0.7);
            sleep(100);
            gyro.gyroDrive(0,0,90,4);
            break;
        }
    }
    private void teleUpdate(){
        telemetry.addData("Brightness = ", robot.cSensor.alpha());
        telemetry.addData("Robot Error = " , gyro.angle_error);
        telemetry.addData("Robot Heading = " , gyro.getAngle());
        telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
        telemetry.update();
    }
}

