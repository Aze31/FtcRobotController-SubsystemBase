package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Bot.getVoltage;


import androidx.core.math.MathUtils;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
public class Drivetrain extends SubsystemBase {
    private ArrayList<DcMotorEx> trainMotors;
    /*below is the universal instance of this class. If you want your class to be a repeatable object, then consider
    not including an instance and making the constructor "public className(){}" This way, you can CREATE new instances
    of your object, which you typically DON'T want to do with your subsystem. */
    private static Drivetrain instance;
    //constructor, every subsystem and command needs one
    public static synchronized Drivetrain getInstance(){
        if(instance == null) instance = new Drivetrain();
        return instance;
    }
    private IMU imu; //IMU tracks rotational displacement
    public double getNormalizedAngle() {
        double angle = getAngle();
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    public Drivetrain(){
        //TODO: CHANGE THE DEVICE NAMES TO WHAT YOUR MOTORS ARE CONFIGURED AS
        trainMotors = new ArrayList<>();
        trainMotors.add(Constants.hardwareMap.get(DcMotorEx.class, "FL"));
        trainMotors.add(Constants.hardwareMap.get(DcMotorEx.class, "BL"));
        trainMotors.add(Constants.hardwareMap.get(DcMotorEx.class, "FR"));
        trainMotors.add(Constants.hardwareMap.get(DcMotorEx.class, "BR"));

        for(DcMotorEx motor : trainMotors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //TODO: enable encoders if you want to use your drive motor encoders
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //TODO: tune these PID coefficients for your drivetrain
        PIDFCoefficients coef = new PIDFCoefficients(0.001,0,0.002,0.001); //remember to tune this
        for(DcMotorEx motor : trainMotors){
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coef);
        }
        imu = Constants.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //the imu stores net rotation of base for the purposes of field-centric drive
    }

    public void driveRC(double x, double y, double rot, boolean slow){
        double yInput = -y;
        x *= -1;
        double slowCoef = 1;
        if(slow){
            slowCoef = 0.25;
        }

        double denominator = Math.max(Math.abs(yInput) + Math.abs(x) + Math.abs(rot), 1);
        double flPower = slowCoef*((yInput + x - rot) / denominator);
        double frPower = slowCoef*((yInput - x + rot) / denominator);
        double blPower = slowCoef*((yInput - x - rot) / denominator);
        double brPower = slowCoef*((yInput + x + rot) / denominator);
        //TODO: if your drivetrain slows down too much on low battery, multiply all four of the below by VoltageComp
        double voltageComp = 12/getVoltage();

        trainMotors.get(0).setPower(flPower);
        trainMotors.get(1).setPower(blPower);
        trainMotors.get(2).setPower(-frPower);
        trainMotors.get(3).setPower(-brPower);
    }
    //---------------------------------------------
    //below is the basic drive method that you'll actually call
    //---------------------------------------------
    public void drive(double xInput, double yInputReversed, double rotInput) {
        double yInput = -yInputReversed;
        double denominator = Math.max(Math.abs(yInput) + Math.abs(xInput) + Math.abs(rotInput), 1);
        double flPower = ((yInput + xInput - rotInput) / denominator);
        double frPower = ((yInput - xInput + rotInput) / denominator);
        double blPower = ((yInput - xInput - rotInput) / denominator);
        double brPower = ((yInput + xInput + rotInput) / denominator);

        trainMotors.get(0).setPower(flPower*12/getVoltage());
        trainMotors.get(1).setPower(blPower*12/getVoltage());
        trainMotors.get(2).setPower(-frPower*12/getVoltage());
        trainMotors.get(3).setPower(-brPower*12/getVoltage());
    }

    //this one drives relative to the field.
    public void driveFC(double y, double x, double rot, boolean slow) {
        // Get the robot's heading in radians
        double heading = Math.toRadians(getNormalizedAngle());

        // Rotate the input vector by -heading to convert to field-centric
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Pass the transformed vector to robot-centric drive
        driveRC(fieldX, fieldY, rot, slow);
    }

    public void stopDrive(){
        for(DcMotorEx motor : trainMotors){
            motor.setPower(0);
        }
    }
    public double getAngle() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void periodic(){}
}