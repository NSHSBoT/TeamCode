package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;




@Autonomous(name="Robot: calibration", group="Robot")
//@Disabled
public class testingCalibrationAuto extends LinearOpMode {


    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    public Servo specimenIntake; // Servo for specimen intake
    public Servo dustpanServo; // Servo for dustpan
    private DcMotorEx linearVerticalSlide = null; // Motor for linear slide (with extended functionality)
    private DcMotorEx intakeArm = null; // Motor for intake arm (with extended functionality)
    private DcMotorEx armRoller = null; // Motor for arm roller (with extended functionality)


    // Constants for movement (These will need to be tuned)
    static final double FORWARD_SPEED = -0.5; // Adjust for desired speed
    static final double STRAFE_SPEED = 0.8;   // Adjust for desired speed
    static final double TURN_SPEED = 0.4;   // Adjust for desired speed


    // Calibration variables (YOU NEED TO CALIBRATE THESE)
    double secondsPerInch = 0.055;   // Time to travel 1 inch forward/backward (example value)
    double secondsPerInchStrafe = 0.05; // Time to strafe 1 inch (example value)
    double secondsPer90Degrees = 0.2; // Time to turn 90 degrees (example value)


    private int GROUND = 0;
    private int SPECIMENPICKUP = 1000; // Example position - adjust as needed
    private int SPECIMENTRANSFER = 2000; // Example position - adjust as needed
    private int SAMPLETRANSFER = 4500;


    private static final double dustpanServoPositionIntaking = 145 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double dustpanServoPositionDumping = 50.0 / 180.0;
    private static final double specimenServoPositionClosed  = 120 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double specimenServoPositionOpen  = 50.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)


    private double liftCurrentEncoderPosition = 0;
    private double armCurrentEncoderPosition = 0;
    private void configureMotor(DcMotorEx motor) { // Helper function to configure motor encoder settings
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Do not use encoder for feedback
    }


    private V4TwoPlayer.LiftState currentLiftState = V4TwoPlayer.LiftState.GROUND;
    private static final int POSITION_DEADBAND = 0;


    private static final double lift_kP = 0.002;
    private static final double liftMotorPower = 1; // Maximum power for the lift motor
    private static final double liftMotorPowerDown = 0.4;


    public void runOpMode() {


        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        linearVerticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        intakeArm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armRoller = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        dustpanServo = hardwareMap.get(Servo.class, "bucket");
        specimenIntake = hardwareMap.get(Servo.class, "claw");
        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Set zero power behavior to BRAKE
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        configureMotor(linearVerticalSlide);
        //linearVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        configureMotor(intakeArm);
        configureMotor(armRoller);
        armRoller.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        dustpanServo.setPosition(dustpanServoPositionIntaking);
        specimenIntake.setPosition(specimenServoPositionOpen);


        // Wait for the start button to be pressed
        waitForStart();


        // Calibration sequence:
        // 1. Move forward 1 foot
        driveForward(12); // 12 inches = 1 foot
        stopDriving();
        sleep(2000);


        // 3. Turn 90 degrees left
//        turnLeft();
//        stopDriving();
//        sleep(2000);
        // 2. Strafe left 1 foot
        strafeLeft(12);  // 12 inches = 1 foot
        stopDriving();
        sleep(2000);




        // Stop the robot
        stopDriving();


//        strafeRight(8.485);
//        driveForward(2);


        while (linearVerticalSlide.getCurrentPosition() < 4480) {
            // this is for arm
            double armPower = calculateSlideP(150);
            intakeArm.setPower(armPower);
            double slidePower = calculateSlideP(4500);
            linearVerticalSlide.setPower(slidePower);
        }


    }


    // Movement methods with calibration variables


    private void strafeLeft(double inches) {
        double time = inches * secondsPerInchStrafe;
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) {
            leftFrontDrive.setPower(STRAFE_SPEED);
            leftBackDrive.setPower(-STRAFE_SPEED);
            rightFrontDrive.setPower(-STRAFE_SPEED);
            rightBackDrive.setPower(STRAFE_SPEED);
        }
    }
    private void strafeRight(double inches) {
        double time = inches * secondsPerInchStrafe;
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) {
            leftFrontDrive.setPower(-STRAFE_SPEED);
            leftBackDrive.setPower(STRAFE_SPEED);
            rightFrontDrive.setPower(-STRAFE_SPEED);
            rightBackDrive.setPower(STRAFE_SPEED);
        }
    }
    public void driveForward(double inches) {
        double time = inches * secondsPerInch;
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) {
            leftFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
        }
    }
    private void turnLeft() {
        runtime.reset();
        while (runtime.seconds() < secondsPer90Degrees && opModeIsActive()) {
            leftFrontDrive.setPower(-TURN_SPEED);
            leftBackDrive.setPower(-TURN_SPEED);
            rightFrontDrive.setPower(TURN_SPEED);
            rightBackDrive.setPower(TURN_SPEED);
        }
    }
    private void turnRight() {
        runtime.reset();
        while (runtime.seconds() < secondsPer90Degrees && opModeIsActive()) {
            leftFrontDrive.setPower(TURN_SPEED);
            leftBackDrive.setPower(TURN_SPEED);
            rightFrontDrive.setPower(-TURN_SPEED);
            rightBackDrive.setPower(-TURN_SPEED);
        }
    }
    private void stopDriving() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    private double calculateSlideP(int targetPosition) {
        // Calculate position error
        double error = targetPosition - liftCurrentEncoderPosition;
        // Return 0 if within deadband to prevent small oscillations
        if (Math.abs(error) <= POSITION_DEADBAND) {
            return 0.0;
        }
        // Calculate P term
        double P = error * lift_kP;
        // Limit the output to the maximum motor power
        if (P > 0) {
            P = Math.min(P, liftMotorPower);
        } else {
            P = Math.max(P, -liftMotorPowerDown);
        }
        return P;
    }


}

