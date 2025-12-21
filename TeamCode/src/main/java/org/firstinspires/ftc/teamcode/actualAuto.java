package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp()
public class actualAuto extends OpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private IMU imu;

    private double speed = 0.5;




    @Override
    public void init()
    {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "LF");
        backLeftMotor = hardwareMap.get(DcMotor.class, "LR");
        frontRightMotor = hardwareMap.get(DcMotor.class, "RF");
        backRightMotor = hardwareMap.get(DcMotor.class, "RR");

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        
        imu.initialize(parameters);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void start() {
        // Begin the 2second forward movement when the OpMode starts
        autoStartTime = getRuntime();
        autoRunning = true;

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }
    
    @Override
    public void loop()
    {
        if (autoRunning) {
            double elapsed = getRuntime() - autoStartTime;

            if (elapsed >= AUTO_DURATION_SEC) {
                // Stop all drive motors
                frontLeftMotor.setPower(0.0);
                backLeftMotor.setPower(0.0);
                frontRightMotor.setPower(0.0);
                backRightMotor.setPower(0.0);

                autoRunning = false;
            }
        }
    }
    @Override
    public void stop() {
        // en sure all motors are stopped when the op stops
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}
