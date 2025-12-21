package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp()
public class mainTeleop extends OpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx shooterMotor;
    private DcMotorSimple intakeMotor, transferMotor;
    //private Servo servoGate;

    private IMU imu;

    FtcDashboard dashboard;
    public static int targetVelocity = 1450;


/* TO DO
    - name servo in control hub
    -


 */


    @Override
    public void init()
    {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "LF");
        backLeftMotor = hardwareMap.get(DcMotor.class, "LR");
        frontRightMotor = hardwareMap.get(DcMotor.class, "RF");
        backRightMotor = hardwareMap.get(DcMotor.class, "RR");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //servoGate = hardwareMap.get(Servo.class, "gate");

        shooterMotor.setPower(0.0);
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        // Set all motors to run using encoders
//        List<DcMotor> motors = Arrays.asList(
//                motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight
//        );
//        for (DcMotor motor : motors) {
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//        // Reverse direction of left-side motors
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //might need to change this one
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(120, 0, 0, 12.5);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Init Complete");


    }
    @Override
    public void loop()
    {
        if (gamepad1.yWasPressed()){
            //from middle
            targetVelocity = 1570;
        }
        if (gamepad1.bWasPressed()){
            //from tip
            targetVelocity = 1450;
        }
        if (gamepad1.aWasPressed()){
            //from back
            targetVelocity = 2100;
        }
        /// /////////
        if (gamepad1.right_trigger > 0){
            shooterMotor.setVelocity(targetVelocity);
            //servoGate.setPosition(ADD VALUE HERE);
        }
        else if (gamepad1.left_trigger > 0){
            shooterMotor.setVelocity(-targetVelocity);
        }
        else{
            shooterMotor.setVelocity(0.0);
            //servoGate.setPosition(ADD VALUE HERE);

        }

        /// ////////////////////////////
        if (gamepad1.right_bumper){
            transferMotor.setPower(1.0);
            intakeMotor.setPower(0.5);
        }
        else if (gamepad1.left_bumper){
            transferMotor.setPower(-1.0);

        }
        else{
            transferMotor.setPower(0.0);
        }
        /// /////////////////
        if (gamepad2.right_trigger > 0){
            intakeMotor.setPower(1.0);
        }
        else if (gamepad2.left_trigger > 0){
            intakeMotor.setPower(-1.0);
        }
        else{
            intakeMotor.setPower(0.0);
        }





        double y = gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad2.left_stick_x;
        double rx = -gamepad2.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        if (gamepad2.back) {
            imu.resetYaw();
        }
        // The equivalent button is start on Xbox-style controllers.

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);


        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("ActualVelocity", shooterMotor.getVelocity());

    }
}
