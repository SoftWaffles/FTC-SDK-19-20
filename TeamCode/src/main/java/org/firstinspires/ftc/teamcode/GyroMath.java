package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class GyroMath {
    //define class members
    ElapsedTime runtime = new ElapsedTime();
    private HardwareTestbot myRobot;

    //declaration of objects
    public Orientation angle;
    public Acceleration gravity;

    //angle variables
    public double globalAngle;
    double target_Angle = 0;
    double prev_angle_error, angle_error;


    //distance variables - LATER
    double distance = 0;
    double prev_dist_error, dist_error;
    double target_Distance = 1;

    //timing
    double elaspsedTime, time, timePrev;
    double period;

    //variables for the PID systems
    double kP = 0.2;
    double kI = 0.2;
    double kD = 1;

    double maxPID = 60;

    //actual PID outputs
    double PID_p, PID_i, PID_d, PID_total;

    public GyroMath(){
    }

    public void initDrive(HardwareTestbot robot){
        myRobot = robot;
        time = runtime.seconds();
    }
    void resetAngle()
    {
        angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    double getAngle()
    {
        Orientation angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = (angle.firstAngle+360)%360;
        return globalAngle;
    }
    public double calcAngle(){
        if(runtime.seconds() > time + period){
            time = runtime.seconds();
            angle_error = target_Angle - getAngle();
            PID_p = kP * angle_error;
            double angle_Derv = angle_error - prev_angle_error;
            PID_d = kD*(angle_Derv/period);

            if(-50 < angle_error && angle_error < 50){
                PID_i = PID_i + (kI * angle_error);
            }else{
                PID_i = 0;
            }
            PID_total = PID_p + PID_i + PID_d;
            prev_angle_error = angle_error;
            //send back value
        }
        PID_total = PID_total/maxPID;
        return PID_total;
    }

}
