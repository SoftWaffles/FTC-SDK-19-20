package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="DRIVE-MODE-ANDRIOD", group="testbot")
//@Disabled
public class TeleTestbot extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    GyroMath gyro = new GyroMath();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware
    boolean wasB = false;
    boolean wasY = false;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        gyro.initDrive(robot);
        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        while (!isStarted()) { telemetry.addData(">", "Robot Heading = ", gyro.getAngle()); telemetry.update();}
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //run loop while button pressed
        while (opModeIsActive() && !isStopRequested()){
            robot.move2D(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            buttons(gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.b, gamepad1.y);
            teleUpdate();
        }
    }
    void buttons(boolean up, boolean down, double LTrig, double RTrig, boolean b, boolean y){
        if(up && !down) {
            robot.lift.setTargetPosition(robot.lift.getCurrentPosition() - 200);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            //while(robot.lift.isBusy()){}
        }
        else if(down && !up) {
            robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + 200);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            //while(robot.lift.isBusy()){}
        } else {
            robot.lift.setTargetPosition(robot.lift.getCurrentPosition());
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            //while(robot.lift.isBusy()){}
        }
        if(y && !wasY){
            if(robot.bar1.getPosition() == 1){
                robot.bar1.setPosition(0);
                robot.bar2.setPosition(0);
            }else {
                robot.bar1.setPosition(1);
                robot.bar2.setPosition(1);
            }
        }
        if(b && !wasB){
            if(robot.grab.getPosition() == 1){
                robot.grab.setPosition(0.6);
            }else{
                robot.grab.setPosition(1);
            }
        }
        wasY = y;
        wasB = b;
    }
    private void teleUpdate(){
        telemetry.addData("Robot Error = " , gyro.angle_error);
        telemetry.addData("Robot Heading = " , gyro.getAngle());
        telemetry.addData("Robot PID Correction = " , gyro.PID_total + " = P( " + gyro.PID_p + " ) + I( " + gyro.PID_i + " ) + D( " + gyro.PID_d + " )");
        telemetry.update();
    }
}