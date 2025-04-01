//new code


package org.firstinspires.ftc.teamcode;

// Importing necessary classes for FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Defining the OpMode as a TeleOp and setting its name and group
@TeleOp(name = "Basic: Omni Linear OpMode", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declaring OpMode members for motors and servos
    private ElapsedTime runtime = new ElapsedTime(); // Timer for tracking runtime
    private DcMotor leftFrontDrive = null; // Motor for left front wheel
    private DcMotor leftBackDrive = null; // Motor for left back wheel
    private DcMotor rightFrontDrive = null; // Motor for right front wheel
    private DcMotor rightBackDrive = null; // Motor for right back wheel

    private DcMotorEx linearVerticalSlide = null; // Motor for linear slide (with extended functionality)
    private DcMotorEx intakeArm = null; // Motor for intake arm (with extended functionality)
    private DcMotorEx armRoller = null; // Motor for arm roller (with extended functionality)

    public Servo intake; // Servo for intake mechanism
    public Servo specimenIntake; // Servo for specimen intake
    public Servo dustpanServo; // Servo for dustpan

    // PID Constants for controlling the intakeArm motor AKA all tunable arm stuff here
    private static final double ARM_KP = 0.00; // Proportional gain
    private static final double ARM_KI = 0.00; // Integral gain
    private static final double ARM_KD = 0.00; // Derivative gain
    private static final double ARM_KG = 0.05; // Gravity compensation coefficient
    private static final double ARM_MAX_OUTPUT = 0.15;
    private static final int ARM_TARGET_HIGH = 350; // Target position for Y button
    private static final int ARM_TARGET_LOW = 0; // Target position for X button
    // State variables for motor control using PID
    private boolean armPIDActive = true; // Flag to enable/disable PID control for the arm
    private boolean slidePIDActive = true; // Flag to enable/disable PID control for the slide
    private int spinnerState = 0; // State of the spinner (0: off, 1: forward, -1: reverse)
    private int currentArmTarget = 0; // Current target position for the arm

    // PID Constants for controlling the linearVerticalSlide motor
    private static final double SLIDE_KP = 0.005; // Proportional gain
    private static final double SLIDE_KI = 0; // Integral gain
    private static final double SLIDE_KD = 0; // Derivative gain

    // Predefined servo positions
    private static final double DUSTPAN_POS_1 = 160.0 / 180.0; // Dustpan position 1 (converted to 0-1 range)
    private static final double DUSTPAN_POS_2 = 70.0 / 180.0; // Dustpan position 2 (converted to 0-1 range)
    private static final double SPECIMEN_POS_1 = 50 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double SPECIMEN_POS_2 = 120.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)

    // State variables to manage toggles and button presses
    private boolean dustpanState = false; // Current state of the dustpan
    private boolean specimenState = false; // Current state of the specimen intake
    private boolean lastRightBumper = false; // Previous state of the right bumper
    private boolean lastLeftBumper = false; // Previous state of the left bumper
    private boolean lastAButton = false; // Previous state of the A button
    private boolean lastBButton = false; // Previous state of the B button



    // Variables for PID controller calculations
    private ElapsedTime pidTimer = new ElapsedTime(); // Timer for PID loop
    private double lastArmError = 0; // Previous error for arm PID
    private double lastSlideError = 0; // Previous error for slide PID
    private double armIntegral = 0; // Accumulated error for arm PID
    private double slideIntegral = 0; // Accumulated error for slide PID

    // Target positions and power limits for motors

    private int currentSlideTarget = 0; // Current target position for the slide
    private static final double MAX_ARM_POWER = 0.3; // Maximum power for the arm motor
    private static final double MAX_SLIDE_POWER = 0.8; // Maximum power for the slide motor
    private static final double SPINNER_POWER = 0.8; // Power for the spinner motor

    // Encoder limits
    private static final int MIN_ARM_POS = 0;
    private static final int MAX_ARM_POS = 500;
    private static final int MIN_LIFT_POS = 0;
    private static final int MAX_LIFT_POS = 6800;

    private static final int POSITION_DEADBAND = 5; // Deadband for PID control
    private static final double INTEGRAL_LIMIT = MAX_ARM_POWER; // Match to max power
    private static final long SERVO_TOGGLE_DELAY = 250; // Milliseconds
    private ElapsedTime servoToggleTimer = new ElapsedTime();
    private boolean limitsEnabled = true; // Emergency override flag
    private boolean lastBackButton = false; // Previous state of the back button


    private static final double ARM_START_ANGLE = 40.0; // Starting angle in degrees
    private static final double ARM_END_ANGLE = 180.0; // End angle in degrees
    private static final double ARM_ENCODER_RANGE = 530.0; // Total encoder ticks
    private double ARM_PID_FF_VALUE = 0;
    private double ARM_PID_OUTPUT = 0;
    private double ARM_MOTOR_OUTPUT_POWER = 0;
    private static final double DEGREES_PER_TICK = (ARM_END_ANGLE - ARM_START_ANGLE) / ARM_ENCODER_RANGE;



    /*
     * This OpMode controls the robot's teleoperated period.
     */
    @Override
    public void runOpMode() {
        // Initialize hardware variables. These correspond to the names given to the devices in the robot configuration file.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        linearVerticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        intakeArm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armRoller = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        dustpanServo = hardwareMap.get(Servo.class, "bucket");
        specimenIntake = hardwareMap.get(Servo.class, "claw");

        // Set motor directions based on robot's physical construction.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Configure encoders for motors requiring precise position control.
        configureMotor(linearVerticalSlide);
        configureMotor(intakeArm);
        configureMotor(armRoller);

        // Initialize servo positions to their starting angles.
        dustpanServo.setPosition(DUSTPAN_POS_1);
        specimenIntake.setPosition(SPECIMEN_POS_2);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart(); // Wait for the game to start.
        runtime.reset(); // Reset the runtime timer.
        pidTimer.reset(); // Reset the timer used for PID calculations.

        while (opModeIsActive()) { // Loop while the OpMode is running.

            handleEmergencyOverride();


            // Handle Dustpan Servo Toggle (Right Bumper - Gamepad 2)
            if (gamepad2.right_bumper && !lastRightBumper && servoToggleTimer.milliseconds() > SERVO_TOGGLE_DELAY) {
                dustpanState = !dustpanState;
                dustpanServo.setPosition(dustpanState ? DUSTPAN_POS_2 : DUSTPAN_POS_1);
                //servoToggleTimer.reset();
            }
            lastRightBumper = gamepad2.right_bumper;

            // Handle Specimen Intake Toggle (Left Bumper - Gamepad 2)
            if (gamepad2.left_bumper && !lastLeftBumper && servoToggleTimer.milliseconds() > SERVO_TOGGLE_DELAY) {
                specimenState = !specimenState;
                specimenIntake.setPosition(specimenState ? SPECIMEN_POS_2 : SPECIMEN_POS_1);
                //servoToggleTimer.reset();
            }
            lastLeftBumper = gamepad2.left_bumper; // Update last button state.

            // Handle Spinner Toggle (A and B Buttons - Gamepad 2)
            // A button sets spinner forward, B button sets spinner reverse.
            if (gamepad2.a && !lastAButton) { // Check for A button press and debounce.
                if (spinnerState == 1) {
                    spinnerState = 0; // Turn off if already running forward.
                } else {
                    spinnerState = 1; // Turn on forward.
                }
            }
            lastAButton = gamepad2.a; // Update last button state.

            if (gamepad2.b && !lastBButton) { // Check for B button press and debounce.
                if (spinnerState == -1) {
                    spinnerState = 0; // Turn off if already running reverse.
                } else {
                    spinnerState = -1; // Turn on reverse.
                }
            }
            lastBButton = gamepad2.b; // Update last button state.

            if (gamepad2.y) { // Move arm to high position
                currentArmTarget = ARM_TARGET_HIGH;
                armPIDActive = true; // Ensure PID is active for controlled movement
            } else if (gamepad2.x) { // Move arm to low position
                currentArmTarget = ARM_TARGET_LOW;
                armPIDActive = true; // Ensure PID is active
            }

            // Apply spinner state to the motor.
            armRoller.setPower(spinnerState * SPINNER_POWER);

            // Handle Arm Control (Left Stick Y - Gamepad 2)
            double armInput = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y); // Square the input for finer control
            if (Math.abs(armInput) > 0.01) { // Reduced deadzone for squared input // Check if stick is moved significantly (deadzone)
                armPIDActive = false; // Disable PID control when manually controlling
                int currentArmPos = intakeArm.getCurrentPosition(); // Get the arm's current position
                // Apply limited power based on position to prevent over-travel
                double limitedArmPower = getLimitedPower(armInput * MAX_ARM_POWER, currentArmPos, MIN_ARM_POS, MAX_ARM_POS);
                intakeArm.setPower(limitedArmPower); // Set power to the arm motor
                ARM_MOTOR_OUTPUT_POWER = limitedArmPower;
            } else if (!armPIDActive) { // If the stick is not moved and PID is not active
                currentArmTarget = intakeArm.getCurrentPosition(); // Set the target position to the current position (hold position)
                armPIDActive = true; // Enable PID control
                armIntegral = 0; // Reset integral term when switching to PID
            }

            // Handle Slide Control (Right Stick Y - Gamepad 2) - Similar logic to Arm Control
            double slideInput = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y); // Square the input for finer control
            if (Math.abs(slideInput) > 0.01) { // Reduced deadzone for squared input // Check for significant stick movement (deadzone)
                slidePIDActive = false; // Disable PID control for manual slide movement
                int currentSlidePos = linearVerticalSlide.getCurrentPosition(); // Get current slide position
                // Apply power limiting based on slide position
                double limitedSlidePower = getLimitedPower(slideInput * MAX_SLIDE_POWER, currentSlidePos, MIN_LIFT_POS, MAX_LIFT_POS);
                linearVerticalSlide.setPower(limitedSlidePower); // Set power to the slide motor
            } else if (!slidePIDActive) { //  If stick isn't moved and PID is inactive
                currentSlideTarget = linearVerticalSlide.getCurrentPosition(); // Set target position to current (hold position)
                slidePIDActive = true; // Activate PID control for the slide
                slideIntegral = 0; // Reset integral term when switching to PID
            }

            // Apply PID control to arm and slide when active.
            if (armPIDActive) { // If arm PID is enabled
                // Calculate PID output for the arm
                double armPower = calculatePID(currentArmTarget, intakeArm.getCurrentPosition(), ARM_KP, ARM_KI, ARM_KD, lastArmError, armIntegral, "arm");
                armPower = Math.max(-MAX_ARM_POWER, Math.min(MAX_ARM_POWER, armPower)); // Limit PID output to max power
                // Apply position limits even to PID output
                armPower = getLimitedPower(armPower, intakeArm.getCurrentPosition(), MIN_ARM_POS, MAX_ARM_POS);
                if (armPower > ARM_MAX_OUTPUT) {armPower = ARM_MAX_OUTPUT;} else if (armPower < -ARM_MAX_OUTPUT) {armPower = -ARM_MAX_OUTPUT;} // Limit power to 0.01
                intakeArm.setPower(armPower); // Set calculated power to the arm motor
                ARM_MOTOR_OUTPUT_POWER = armPower;
            }

            if (slidePIDActive) { // If slide PID is enabled
                // Calculate PID output for the slide
                double slidePower = calculatePID(currentSlideTarget, linearVerticalSlide.getCurrentPosition(), SLIDE_KP, SLIDE_KI, SLIDE_KD, lastSlideError, slideIntegral, "slide");
                slidePower = Math.max(-MAX_SLIDE_POWER, Math.min(MAX_SLIDE_POWER, slidePower)); // Constrain PID output within max power
                // Apply position limits to the PID output as well
                slidePower = getLimitedPower(slidePower, linearVerticalSlide.getCurrentPosition(), MIN_LIFT_POS, MAX_LIFT_POS);
                linearVerticalSlide.setPower(slidePower); // Set calculated power to the slide motor
            }

      // Driving controls (Gamepad 1) with added cubic scaling and speed reduction
            double axial, lateral, yaw;

            if (gamepad1.left_bumper || gamepad1.right_bumper) { // Check for either bumper pressed
                axial = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * Math.abs(gamepad1.left_stick_y) * 0.5; // Cubic scaling and speed reduction
                lateral = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * Math.abs(gamepad1.left_stick_x) * 0.5; // Cubic scaling and speed reduction
                yaw = -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * Math.abs(gamepad1.right_stick_x) * 0.5; // Cubic scaling and speed reduction
            } else { // Regular squared scaling
                axial = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // squared for finer control
                lateral = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
                yaw = -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
            }

            // Calculate individual wheel powers based on driver input.
            double[] powers = calculateWheelPowers(axial, lateral, yaw);

            // Set drive motor powers.
            leftFrontDrive.setPower(powers[0]);
            rightFrontDrive.setPower(powers[1]);
            leftBackDrive.setPower(powers[2]);
            rightBackDrive.setPower(powers[3]);

            // Display telemetry data for debugging and driver feedback.
            telemetry.addData("LIMITS", limitsEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Arm Position", intakeArm.getCurrentPosition());
            telemetry.addData("Arm Target", currentArmTarget);
            telemetry.addData("Arm PID Active", armPIDActive);
            telemetry.addData("Arm PID FF Value", ARM_PID_FF_VALUE);
            telemetry.addData("Arm Motor Power",ARM_MOTOR_OUTPUT_POWER);
            telemetry.addData("Arm Motor PID Power", ARM_PID_OUTPUT);
            telemetry.addData("Slide Position", linearVerticalSlide.getCurrentPosition());
            telemetry.addData("Slide Target", currentSlideTarget);
            telemetry.addData("Slide PID Active", slidePIDActive);
            telemetry.addData("Dustpan State", dustpanState ? "90°" : "45°");
            telemetry.addData("Specimen State", specimenState ? "90°" : "45°");
            telemetry.addData("Spinner State", spinnerState == 0 ? "OFF" : (spinnerState == 1 ? "FORWARD" : "REVERSE"));
            telemetry.addData("leftFrontDrivePower",powers[0]);

            telemetry.update();
        }
    }

    // Helper function to configure motor encoder settings.
    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder for feedback.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motor when power is zero.
    }

    private double getArmAngle(int encoderPosition) {
        return ARM_START_ANGLE + (encoderPosition * DEGREES_PER_TICK);
    }

    private double calculateArmFeedforward(int currentPosition) {

        double currentAngle = getArmAngle(currentPosition);
        double angleRadians = Math.toRadians(currentAngle);
        //ARM_PID_FF_VALUE = ARM_KG*Math.cos(angleRadians);
        return ARM_KG * Math.cos(angleRadians);
    }

    private double calculatePID(int targetPosition, int currentPosition, double kP, double kI, double kD, double lastError, double integral, String motorType) {
        double error = targetPosition - currentPosition;

        if (Math.abs(error) <= POSITION_DEADBAND) {
            return 0.0;
        }

        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        double P = error * kP;

        integral += error * deltaTime;
        integral = Math.max(-1.0, Math.min(1.0, integral));
        double I = integral * kI;

        double derivative = (error - lastError) / deltaTime;
        if (motorType.equals("arm")) {
            derivative = derivative * 0.7;
        }
        double D = derivative * kD;

        if (motorType.equals("arm")) {
            lastArmError = error;
            double feedforward = calculateArmFeedforward(currentPosition);
            ARM_PID_OUTPUT = P+I+D;
            ARM_PID_FF_VALUE = P+I+D+feedforward;
            return P + I + D + feedforward;
        } else {
            lastSlideError = error;
            return P + I + D;
        }
    }

    private double[] calculateWheelPowers(double axial, double lateral, double yaw) {
        double[] powers = new double[4];

        powers[0] = (axial - lateral + yaw) * 0.5; // Left Front
        powers[1] = (axial + lateral - yaw) * 0.5; // Right Front
        powers[2] = (axial + lateral + yaw) * 0.5; // Left Back
        powers[3] = (axial - lateral - yaw) * 0.5; // Right Back

        double max = Math.abs(powers[0]);
        for (int i = 1; i < 4; i++) {
            max = Math.max(max, Math.abs(powers[i])); // Find the maximum absolute wheel power.
        }

        if (max > 1.0) {

            for (int i = 0; i < 4; i++) {
                powers[i] /= max; // Normalize wheel powers if any exceed 1.0.
            }
        }

        return powers;
    }

    private double getLimitedPower(double inputPower, int currentPosition, double minLimit, double maxLimit) {
        if (!limitsEnabled) {
            return inputPower; // Bypass limits if disabled
        }

        // Convert position to a 0-1 range where 0 is minLimit and 1 is maxLimit
        double positionRatio = (currentPosition - minLimit) / (maxLimit - minLimit);

        // Limit movement based on position
        if (positionRatio <= 0) {
            return Math.max(0, inputPower);
        } else if (positionRatio >= 1) {
            return Math.min(0, inputPower);
        }
        return inputPower;
    }

    // Add this method to handle emergency override
    private void handleEmergencyOverride() {
        if (gamepad2.back && !lastBackButton) {
            limitsEnabled = !limitsEnabled;

        }
        lastBackButton = gamepad2.back;
    }
}

