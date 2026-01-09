package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp()
public class servoTester extends OpMode {
    private Servo servo;
    public static double position = 0;
    FtcDashboard dashboard;
    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, "gate");
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Init Complete");
    }
    public void loop(){
        servo.setPosition(position);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.addData("servo min", servo.MIN_POSITION);
        telemetry.addData("servo max", servo.MAX_POSITION);

    }

}
