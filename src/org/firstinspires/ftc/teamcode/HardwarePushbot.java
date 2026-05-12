package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * HardwarePushbot - DECODE Season Hardware Mapping
 *
 * Hardware Configuration:
 * - Drive: 4 Mecanum wheels (FL, FR, BL, BR)
 * - Odometry: 3 Dead Wheels for 2D localization
 * - Intake: Continuous rotation DC motor
 * - Shooter: Flywheel motor (launcher) with encoder
 * - Gate: Servo for Overflow release
 * - Sensors: Distance sensor for Classifier Ramp
 * - Vision: Webcam for AprilTag detection
 */
public class HardwarePushbot {

    // Drive Motors (Mecanum)
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // Dead Wheel Odometry
    public DcMotor deadWheelLeft = null;
    public DcMotor deadWheelRight = null;
    public DcMotor deadWheelCenter = null;

    // Mechanisms
    public DcMotor intakeMotor = null;
    public DcMotor launcher = null;
    public Servo gateServo = null;

    // Sensors
    public DistanceSensor distanceSensor = null;
    public VoltageSensor voltageSensor = null;

    // Vision
    public WebcamName webcam = null;

    // Constants
    public static final double MECANUM_WHEEL_DIAMETER_INCHES = 4.0;
    public static final double DEAD_WHEEL_DIAMETER_INCHES = 2.0;
    public static final int ENCODER_TICKS_PER_REV = 5376; // REV UltraPlanetary
    public static final double INTAKE_POWER = 0.8;
    public static final double LAUNCHER_TARGET_RPM = 3000;
    public static final double GATE_OPEN_POSITION = 0.7;
    public static final double GATE_CLOSED_POSITION = 0.0;

    private HardwareMap hardwareMap = null;

    public HardwarePushbot() {
    }

    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Drive Motors - mecanum configuration
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        configureDriveMotors();

        // Dead Wheel Odometry
        deadWheelLeft = hwMap.get(DcMotor.class, "deadWheelLeft");
        deadWheelRight = hwMap.get(DcMotor.class, "deadWheelRight");
        deadWheelCenter = hwMap.get(DcMotor.class, "deadWheelCenter");

        configureDeadWheels();

        // Mechanisms
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        launcher = hwMap.get(DcMotor.class, "launcher");
        gateServo = hwMap.get(Servo.class, "gateServo");

        configureMechanisms();

        // Sensors
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        voltageSensor = hwMap.get(VoltageSensor.class, "voltageSensor");

        // Vision
        webcam = hwMap.get(WebcamName.class, "webcam");
    }

    private void configureDriveMotors() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configureDeadWheels() {
        deadWheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        deadWheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        deadWheelCenter.setDirection(DcMotorSimple.Direction.REVERSE);

        deadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        deadWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deadWheelCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configureMechanisms() {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gateServo.setPosition(GATE_CLOSED_POSITION);
    }

    /**
     * Get current battery voltage for PIDF feedforward
     */
    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        }
        return 12.0; // Default fallback
    }

    /**
     * Get distance from classifier ramp sensor
     */
    public double getRampDistance() {
        if (distanceSensor != null) {
            return distanceSensor.getDistance(com.qualcomm.robotcore.hardware.UnitSystem.METRIC);
        }
        return Double.MAX_VALUE;
    }

    /**
     * Set all drive motors to same power (for testing)
     */
    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Stop all drive motors
     */
    public void stopDrive() {
        setDrivePower(0);
    }

    /**
     * Open the gate to release Overflow to Secret Tunnel
     */
    public void openGate() {
        gateServo.setPosition(GATE_OPEN_POSITION);
    }

    /**
     * Close the gate
     */
    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED_POSITION);
    }

    /**
     * Start intake motor to collect artifacts
     */
    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /**
     * Stop intake motor
     */
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    /**
     * Start launcher at specified power
     */
    public void startLauncher(double power) {
        launcher.setPower(power);
    }

    /**
     * Stop launcher
     */
    public void stopLauncher() {
        launcher.setPower(0);
    }
}