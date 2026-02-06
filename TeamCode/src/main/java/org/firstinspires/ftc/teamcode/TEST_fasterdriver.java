package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class  TEST_fasterdriver extends OpMode {
    // Declare hardware variables
    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private BNO055IMU imu;

    private DcMotorEx shooterMotor;
    private DcMotorSimple intakeMotor, transferMotor;
    private Servo servoGate;
    FtcDashboard dashboard;
    public static int targetVelocity = 1350;



    // Variables for scaling and gamepad state
    private double[] speedModes = {0.3, 0.55, 0.8, 1.0};
    private int currentSpeedModeIndex = 1; // default to 0.55
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private double dpadY, dpadX;


    @Override
    public void init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "LF");
        motorBackLeft = hardwareMap.get(DcMotor.class, "LR");
        motorFrontRight = hardwareMap.get(DcMotor.class, "RF");
        motorBackRight = hardwareMap.get(DcMotor.class, "RR");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        servoGate = hardwareMap.get(Servo.class, "gate");


        // Set all motors to run using encoders
        List<DcMotor> motors = Arrays.asList(
                motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight
        );
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Reverse direction of left-side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        shooterMotor.setPower(0.0);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //might need to change this one
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(120, 0, 0, 12.5);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {


        /// /////////
        if (gamepad1.right_trigger > 0){
            shooterMotor.setVelocity(targetVelocity);
            servoGate.setPosition(0.6);

        }
        else if (gamepad1.left_trigger > 0){
            shooterMotor.setVelocity(-targetVelocity);
        }
        else{
            shooterMotor.setVelocity(0.0);
            servoGate.setPosition(0.25);

        }
        if (gamepad1.dpadUpWasPressed()){
            targetVelocity += 50;
        }
        if (gamepad1.dpadDownWasPressed()){
            targetVelocity -= 50;
        }

        /// ////////////////////////////
        if (gamepad1.right_bumper){
            transferMotor.setPower(1.0);
            intakeMotor.setPower(1.0);

        }
        else if (gamepad1.left_bumper){
            transferMotor.setPower(-1.0);

        }
        else{
            transferMotor.setPower(0.0);
        }
        /// /////////////////
        if (gamepad2.right_trigger > 0 || gamepad2.right_bumper){
            intakeMotor.setPower(1.0);
            transferMotor.setPower(1.0);
        }
        else if (gamepad2.left_trigger > 0 || gamepad2.left_bumper){
            intakeMotor.setPower(-1.0);
        }
        else{
            intakeMotor.setPower(0.0);
        }

        // Overly complicated speed mode adjustment logic
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;


        int adjustment = (currentDpadUp ? 1 : 0) - (currentDpadDown ? 1 : 0);
        boolean stateChanged = currentDpadUp != prevDpadUp || currentDpadDown != prevDpadDown;

        if (stateChanged) {
            int tentativeIndex = currentSpeedModeIndex;
            for (int i = 0; i < Math.abs(adjustment); i++) {
                if (adjustment > 0 && tentativeIndex < speedModes.length - 1) {
                    tentativeIndex++;
                } else if (adjustment < 0 && tentativeIndex > 0) {
                    tentativeIndex--;
                }
            }
            currentSpeedModeIndex = tentativeIndex;
        }

        // Update previous button states
        prevDpadUp = currentDpadUp;
        prevDpadDown = currentDpadDown;

        double speedScale = speedModes[currentSpeedModeIndex];

        double forwardTrigger = gamepad1.right_trigger;
        double backwardTrigger = gamepad1.left_trigger;

        double botOrientedY = 0.0;
        if (forwardTrigger > 0) {
            botOrientedY = forwardTrigger;
        } else if (backwardTrigger > 0) {
            botOrientedY = -backwardTrigger;
        }

        if (gamepad2.dpad_up){
            dpadY = -0.2;
        } else if (gamepad2.dpad_down) {
            dpadY = 0.2;
        }
        else dpadY = 0;
        if (gamepad2.dpad_right){
            dpadX = 0.2;
        } else if (gamepad2.dpad_left) {
            dpadX = -0.2;
        }
        else dpadX = 0;

        // NORMAL FIELD-ORIENTED JOYSTICK MOVEMENT
        double y = -(gamepad2.left_stick_y + dpadY);
        double x = (gamepad2.left_stick_x + dpadX) * 1.1;
        double rx = gamepad2.right_stick_x;

        double botHeadingRadians = -imu.getAngularOrientation().firstAngle;

        // Apply field-centric transformation for joystick movement
        double rotX = x * Math.cos(botHeadingRadians) - y * Math.sin(botHeadingRadians);
        double rotY = x * Math.sin(botHeadingRadians) + y * Math.cos(botHeadingRadians);

        // Combine bot-oriented trigger control with field-oriented joystick control
        double finalY = botOrientedY != 0.0 ? botOrientedY : rotY;

        double denominator = Math.max(Math.abs(finalY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower = (finalY + rotX + rx) / denominator;
        double backLeftPower = (finalY - rotX + rx) / denominator;
        double frontRightPower = (finalY - rotX - rx) / denominator;
        double backRightPower = (finalY + rotX - rx) / denominator;

        boolean isTurboMode = gamepad1.right_bumper;
        double adjustedSpeedScale = isTurboMode ? 1.0 : speedScale;

        frontLeftPower *= adjustedSpeedScale;
        backLeftPower *= adjustedSpeedScale;
        frontRightPower *= adjustedSpeedScale;
        backRightPower *= adjustedSpeedScale;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);



        // Send telemetry data to driver station
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Speed Mode", speedScale);
        telemetry.addData("Heading", Math.toDegrees(botHeadingRadians));
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("ActualVelocity", shooterMotor.getVelocity());
        telemetry.update();
    }
}
