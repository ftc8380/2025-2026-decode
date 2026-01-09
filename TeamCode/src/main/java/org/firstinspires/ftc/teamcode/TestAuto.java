package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    public class Shooter {
        private DcMotorEx shooterMotor;

        public Shooter(HardwareMap hardwareMap) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setPower(0.0);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //might need to change this one
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(120, 0, 0, 12.5);
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }

        public class ShooterOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor
                    shooterMotor.setVelocity(1400);
                return false;
            }
        }

        public class ShooterOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor
                shooterMotor.setPower(0);
                return false;
            }
        }

        public Action shooterOn() {
            return new ShooterOn();
        }

        public Action shooterOff() {
            return new ShooterOff();
        }

    }

    public class Transfer {

        private DcMotorSimple transferMotor;
        private Servo servoGate;
        public Transfer(HardwareMap hardwareMap) {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer");
            servoGate = hardwareMap.get(Servo.class, "gate");
        }

        public class ServoOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // opens servo Gate
                servoGate.setPosition(0.6);
                return false;
            }
        }

        public Action servoOpen() {
            return new ServoOpen();
        }

        public class ServoClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // opens servo Gate
                servoGate.setPosition(0.25);
                return false;
            }
        }

        public Action servoClose() {
            return new ServoClose();
        }

        public class TransferIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // turns on transfer so balls go toward shooter
                transferMotor.setPower(1.0);
                return false;
            }
        }

        public Action transferIn() {
            return new TransferIn();
        }
    }

    public class Intake {

        private DcMotorSimple intakeMotor;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        }

        public class IntakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // turns on transfer so balls go toward shooter
                intakeMotor.setPower(1.0);
                return false;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // turns on transfer so balls go toward shooter
                intakeMotor.setPower(0);
                return false;
            }
        }

        public Action IntakeOff() {
            return new IntakeOff();
        }
    }



    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a shooter instance
        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12, -12));

        Action tab2 = tab1.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(-12, -50))
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(225))
                .build();

        Action shootBalls = new SequentialAction(
                transfer.servoOpen(),
                transfer.transferIn(),
                intake.intakeIn(),
                new SleepAction(5),
                shooter.shooterOff()
        );


        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shooter.shooterOn(),
                        tab1.build(),
                        shootBalls,
                        transfer.servoClose(),
                        tab2,
                        shootBalls,
                        transfer.servoClose()
                )
        );
    }

}

