package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
@TeleOp()
public class PIDTester extends OpMode {
    public DcMotorEx shooterMotor;
    public static double targetVelocity = 1000;
    public static double currentVelocity;
    //120
    public static double P;
    public static double I;
    public static double D;
    //12.5
    public static double F;
    boolean motorRunning;
    FtcDashboard dashboard;

    int stepIndex = 1;
    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //might need to change this one
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Init Complete");
    }


    public void loop(){

        // sets new pidf coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // setting velocity
        if (gamepad1.a){
            shooterMotor.setVelocity(targetVelocity);
        }
        else {
            shooterMotor.setVelocity(0.0);
        }
        currentVelocity = shooterMotor.getVelocity();
        double error = targetVelocity - currentVelocity;



        telemetry.addData("Target velocity", targetVelocity);
        telemetry.addData("Current velocity", currentVelocity);

    }
}

