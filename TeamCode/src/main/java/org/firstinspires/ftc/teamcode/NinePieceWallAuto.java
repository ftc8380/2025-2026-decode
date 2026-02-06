package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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

import java.util.Arrays;


@Config
@Autonomous(name = "NinePieceWallAuto", group = "Autonomous")
public class NinePieceWallAuto extends LinearOpMode {

    // Top of your OpMode
    boolean isBlue = true; // Change to false for Red
    double flip = isBlue ? 1 : -1;
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
                    shooterMotor.setVelocity(1450);
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

        public Action transferOff() {
            return new TransferIn();
        }

        public class TransferOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // turns on transfer so balls go toward shooter
                transferMotor.setPower(1.0);
                return false;
            }
        }

        public Action transferIn() {
            return new TransferOff();
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

        public Action intakeOff() {
            return new IntakeOff();
        }
    }



    @Override
    public void runOpMode() {
        // 2. SELECTION LOOP: This runs AFTER you hit INIT but BEFORE you hit START
        while (!isStarted() && !isStopRequested() && !(gamepad1.a)) {
            if (gamepad1.x) isBlue = true;  // X button for Blue
            if (gamepad1.b) isBlue = false; // B button for Red

            flip = isBlue ? 1 : -1;

            // Show the current selection on the Driver Station phone
            telemetry.addLine("Press X for Blue, B for Red and A to finalize selection");
            telemetry.update();
        }

        telemetry.addData("SELECTED SIDE IS FINALIZED", isBlue ? "BLUE (X)" : "RED (B)");
        telemetry.update();

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-50, -50 * flip, Math.toRadians(225) * flip);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a shooter instance
        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double shootTime = 2.5; //seconds

        VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45.0),
                new AngularVelConstraint(Math.PI)
        ));



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12, -12 * flip));

        TrajectoryActionBuilder tab2 = tab1.fresh()
                .turnTo(Math.toRadians(270) * flip)
                .strafeTo(new Vector2d(-12, -55  * flip))
                .stopAndAdd(shooter.shooterOn())
                .strafeToLinearHeading(new Vector2d(-12, -12  * flip), Math.toRadians(225)  * flip);

        TrajectoryActionBuilder tab3 = tab2.fresh()
//                .turnTo(Math.toRadians(270))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, -24 * flip, Math.toRadians(270) * flip), Math.toRadians(270)  * flip)
                .strafeToLinearHeading(new Vector2d(12, -60 * flip), Math.toRadians(270) * flip, slowVelConstraint)
                .stopAndAdd(shooter.shooterOn())
                .strafeToLinearHeading(new Vector2d(-12, -12 * flip), Math.toRadians(225) * flip);

        TrajectoryActionBuilder tab4 = tab3.fresh()
//                .turnTo(Math.toRadians(270))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(36, -24 * flip, Math.toRadians(270) * flip), Math.toRadians(270) * flip)
                .strafeToLinearHeading(new Vector2d(36, -60 * flip), Math.toRadians(270) * flip, slowVelConstraint)
                .stopAndAdd(shooter.shooterOn())
                .strafeToLinearHeading(new Vector2d(-12, -12 * flip), Math.toRadians(225) * flip);

//        public SequentialAction shootBalls() {
//            return new SequentialAction(
//                    transfer.servoOpen(),
//                    transfer.transferIn(),
//                    intake.intakeIn(),
//                    new SleepAction(3),
//                    shooter.shooterOff(),
//                    transfer.servoClose());
//        }

        Action shootBalls = new SequentialAction(
                transfer.servoOpen(),
                transfer.transferIn(),
                intake.intakeIn(),
                new SleepAction(5),
                shooter.shooterOff(),
                transfer.servoClose()
            );

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shooter.shooterOn(),
                        tab1.build(),
                        //shoots Balls
                        new SequentialAction(
                            transfer.servoOpen(),
                            transfer.transferIn(),
                            intake.intakeIn(),
                            new SleepAction(shootTime),
                            shooter.shooterOff(),
                            transfer.servoClose()
                        ),
                        tab2.build(),
                        new SequentialAction(
                            transfer.servoOpen(),
                            transfer.transferIn(),
                            intake.intakeIn(),
                            new SleepAction(shootTime),
                            shooter.shooterOff(),
                            transfer.servoClose()
                        ),
                        tab3.build(),
                        new SequentialAction(
                                transfer.servoOpen(),
                                transfer.transferIn(),
                                intake.intakeIn(),
                                new SleepAction(shootTime),
                                shooter.shooterOff(),
                                transfer.servoClose()
                        ),
                        tab4.build(),
                        new SequentialAction(
                                transfer.servoOpen(),
                                transfer.transferIn(),
                                intake.intakeIn(),
                                new SleepAction(shootTime),
                                shooter.shooterOff(),
                                transfer.servoClose()
                        ),
                        new SequentialAction(
                                transfer.servoClose(),
                                transfer.transferOff(),
                                intake.intakeOff(),
                                shooter.shooterOff()
                        )
                )
        );
    }

}

