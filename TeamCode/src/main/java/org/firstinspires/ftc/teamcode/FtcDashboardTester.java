package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

//@Config needed for ftc dashboard
@Config
@TeleOp()
public class FtcDashboardTester extends OpMode {
    FtcDashboard dashboard;
    ColorSensor colorSensor;

    //variable that shows up on the dashboard (public static needed for it to show up on FTC Dashboard)
    // you can now change their values on the FTC Dashboard
    public static boolean TESTER_VARIABLE;
    @Override
    public void init(){

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


    }
    public void loop(){
        telemetry.addData("TESTER_VARIABLE", TESTER_VARIABLE);
        telemetry.addData("Red", colorSensor.red());

    }
}
