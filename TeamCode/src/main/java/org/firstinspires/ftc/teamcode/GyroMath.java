package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class GyroMath {
    //define class members
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareTestbot myRobot;
    //angle variables
    private double globalAngle;
    double target_Angle = 0;
    private double prev_angle_error = 0;
    double angle_error;
    //timing
    private double time;
    private double period = 1;
    //variables for the PID systems
    double kP = 0.005;
    double kI = 0.001;
    double kD = 0.05;
    //actual PID outputs
    double PID_p, PID_i = 0, PID_d = 0, PID_total;

    public GyroMath() { }

    public void initDrive(HardwareTestbot robo) {
        myRobot = robo;
        time = runtime.seconds();
    }
    public void gyroDrive(double forw, double side, double target, int time){
        runtime.reset();
        while(runtime.seconds()<time){
            if(getError(target) < 3){
                myRobot.move2D(forw,side,0);
            }
            myRobot.move2D(0,0,calcPID(target));
        }
        myRobot.move2D(0,0,0);
    }
    //PID Math given target
    public double calcPID(double target){
        target_Angle = target;
        if(runtime.seconds() > time + period){
            time = runtime.seconds();
            //CONVENTIONS USED COUNTERCLOCKWISE IS NEGATIVE TURN ----- CLOCKWISE IS POSITIVE TURN
            angle_error = getError(target_Angle);
            PID_p = kP * angle_error;

            double angle_Derv = angle_error - prev_angle_error;
            PID_d = kD*(angle_Derv/period);
            /*
            if(-50 > angle_error && angle_error < 50){
                PID_i = PID_i + (kI * angle_error);
            }else{
                PID_i = 0;
            }
            */
            if(Math.abs(angle_error) > 5){
                PID_total = PID_p; //+ PID_i + PID_d;
            }else{
                PID_total = 0;
            }
            prev_angle_error = angle_error;
        }
        return PID_total;
    }

    double getError(double target){
        return target - getGlobalAngle();
    }
    //make current heading the zero
    void resetAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    //reading angle objects z axis
    public double getAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return myRobot.angle.firstAngle;
    }
    //converting heading to global angle
    double getGlobalAngle() {
        myRobot.angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = (myRobot.angle.firstAngle+360)%360;
        return globalAngle;
    }
    //convert target to hemisphere angle
    double convertToHemi(double target){
        double hemiTarget = target;
        if(target > 179){
            hemiTarget = ((target % 180)-180);
        }
        return hemiTarget;
    }
}
