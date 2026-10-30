package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter & Intake Control", group = "TeleOp")
public class ShooterMechanismTeleOp extends LinearOpMode {

    // Flywheel constants (1:1 Yellow Jacket / bare motor = 28 ticks/rev)
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double TARGET_FLYWHEEL_RPM = 3000.0;
    private static final double TARGET_FLYWHEEL_TICKS_PER_SEC = (TARGET_FLYWHEEL_RPM / 60.0) * FLYWHEEL_TICKS_PER_REV; // 1400.0 ticks/sec

    // Servo positions
    private static final double SERVO1_POS_POLLEN = 0.4;
    private static final double SERVO1_POS_CLOSED = 0.95;

    private static final double SERVO2_POS_POLLEN_SHOOT = 0.95; // Initial/Home pose
    private static final double SERVO2_POS_NECTAR_SHOOT = 0.5;

    // Hardware devices
    private Servo axonServo1;
    private Servo axonServo2;
    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private DcMotorEx intakeMotor;

    // Toggle state variables
    private boolean servo1PollenState = false; // false = Closed Gate (0.95), true = Pollen (0.4)
    private boolean servo2NectarState = false; // false = Pollen Shoot Home (0.95), true = Nectar Shoot (0.5)
    private boolean flywheelOn = false;
    private boolean intakeOn = false;

    // Gamepad button edge detection (previous button state)
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        axonServo1 = hardwareMap.get(Servo.class, "axonServo1");
        axonServo2 = hardwareMap.get(Servo.class, "axonServo2");

        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Prepare flywheel encoders
        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Prepare intake motor
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial servo positions
        axonServo1.setPosition(SERVO1_POS_CLOSED); // Default Closed Gate (0.95)
        axonServo2.setPosition(SERVO2_POS_POLLEN_SHOOT); // Initial/Home pose (0.95)

        telemetry.addLine("Shooter & Intake System Initialized.");
        telemetry.addLine("X: Toggle Axon Servo 1 (0.4 Pollen / 0.95 Closed)");
        telemetry.addLine("B: Toggle Axon Servo 2 (0.95 Home / 0.5 Nectar)");
        telemetry.addLine("Y: Toggle Flywheels (3000 RPM)");
        telemetry.addLine("A: Toggle Intake (Full Speed)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- X BUTTON TOGGLE: Axon Servo 1 ---
            if (gamepad1.x && !lastX) {
                servo1PollenState = !servo1PollenState;
            }
            lastX = gamepad1.x;

            double targetServo1Pos = servo1PollenState ? SERVO1_POS_POLLEN : SERVO1_POS_CLOSED;
            axonServo1.setPosition(targetServo1Pos);


            // --- B BUTTON TOGGLE: Axon Servo 2 ---
            if (gamepad1.b && !lastB) {
                servo2NectarState = !servo2NectarState;
            }
            lastB = gamepad1.b;

            double targetServo2Pos = servo2NectarState ? SERVO2_POS_NECTAR_SHOOT : SERVO2_POS_POLLEN_SHOOT;
            axonServo2.setPosition(targetServo2Pos);


            // --- Y BUTTON TOGGLE: Flywheel Motors (3000 RPM) ---
            if (gamepad1.y && !lastY) {
                flywheelOn = !flywheelOn;
            }
            lastY = gamepad1.y;

            if (flywheelOn) {
                flywheelMotor1.setVelocity(TARGET_FLYWHEEL_TICKS_PER_SEC);
                flywheelMotor2.setVelocity(TARGET_FLYWHEEL_TICKS_PER_SEC);
            } else {
                flywheelMotor1.setVelocity(0);
                flywheelMotor2.setVelocity(0);
            }


            // --- A BUTTON TOGGLE: Intake Motor (Full Speed) ---
            if (gamepad1.a && !lastA) {
                intakeOn = !intakeOn;
            }
            lastA = gamepad1.a;

            if (intakeOn) {
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
            }


            // --- TELEMETRY ---
            double currentRPM1 = (flywheelMotor1.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0;
            double currentRPM2 = (flywheelMotor2.getVelocity() / FLYWHEEL_TICKS_PER_REV) * 60.0;

            telemetry.addData("Servo 1 State", servo1PollenState ? "Pollen (0.4)" : "Closed Gate (0.95)");
            telemetry.addData("Servo 2 State", servo2NectarState ? "Nectar Shoot (0.5)" : "Pollen Shoot Home (0.95)");
            telemetry.addData("Flywheels State", flywheelOn ? "ON (3000 RPM)" : "OFF");
            telemetry.addData("Flywheel 1 Actual RPM", "%.1f", currentRPM1);
            telemetry.addData("Flywheel 2 Actual RPM", "%.1f", currentRPM2);
            telemetry.addData("Intake State", intakeOn ? "ON (100%)" : "OFF");
            telemetry.update();
        }

        // Safety shutdown
        flywheelMotor1.setVelocity(0);
        flywheelMotor2.setVelocity(0);
        intakeMotor.setPower(0);
    }
}
