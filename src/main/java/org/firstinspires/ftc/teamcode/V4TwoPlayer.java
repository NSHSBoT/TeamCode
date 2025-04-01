package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "A-V4 TWO Player", group = "Linear OpMode")
public class V4TwoPlayer extends LinearOpMode {


    // #region Variables
    // #region Declarations
    private ElapsedTime runtime = new ElapsedTime(); // Timer for tracking runtime
    private DcMotor leftFrontDrive = null; // Motor for left front wheel
    private DcMotor leftBackDrive = null; // Motor for left back wheel
    private DcMotor rightFrontDrive = null; // Motor for right front wheel
    private DcMotor rightBackDrive = null; // Motor for right back wheel


    private DcMotorEx linearVerticalSlide = null; // Motor for linear slide (with extended functionality)
    private DcMotorEx intakeArm = null; // Motor for intake arm (with extended functionality)
    private DcMotorEx armRoller = null; // Motor for arm roller (with extended functionality)


    public Servo specimenIntake; // Servo for specimen intake
    public Servo dustpanServo; // Servo for dustpan
    // #endregion Declarations


    // #region Uneditable Variables
    private boolean dustpanState = false; // Current state of the dustpan
    private boolean specimenState = true; // Current state of the specimen intake
    private boolean lastRightBumper = false; // Previous state of the right bumper
    private boolean lastLeftBumper = false; // Previous state of the left bumper
    private boolean lastAButton = false; // Previous state of the A button
    private boolean lastBButton = false; // Previous state of the B button
    private boolean lastXButton = false;
    private boolean lastYButton = false;
    private boolean armPIDActive = true; // Flag to enable/disable PID control for the arm
    private boolean slidePIDActive = true; // Flag to enable/disable PID control for the slide
    private double spinnerState = 0; // State of the spinner (0: off, 1: forward, -1: reverse)
    private ElapsedTime pidTimer = new ElapsedTime(); // Timer for PID loop
    private double lastArmError = 0; // Previous error for arm PID
    private double lastSlideError = 0; // Previous error for slide PID
    private double armIntegral = 0; // Accumulated error for arm PID
    private double slideIntegral = 0; // Accumulated error for slide PID
    private ArmState currentArmState = ArmState.IDLE;
    private LiftState currentLiftState = LiftState.GROUND;
    private boolean limitsEnabled = true; // Emergency override flag
    private boolean lastBackButton = false; // Previous state of the back button
    private double ARM_PID_FF_VALUE = 0;
    private double ARM_PID_OUTPUT = 0;
    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;
    int currentArmPos = 0;
    double armPower = 0;
    double limitedArmPower = 0;
    double liftInput = 0;
    int currentSlidePos = 0;
    double slidePower = 0;
    double limitedSlidePower = 0;
    boolean reducedSpeed = false;
    boolean turboMode = false;
    double speedMultiplier;
    double[] powers;
    private boolean dpad_right_pressed = false;
    private boolean dpad_left_pressed = false;
    private double liftCurrentEncoderPosition = 0;
    private double armCurrentEncoderPosition = 0;
    private double liftPIDValue;


    //private RobotState currentRobotState = RobotState.IDLE;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastStartButton = false;
    private boolean firstArmMotorCalibrate = true;


    // #endregion Uneditable Variables


    // #region Editable Variables


    // #region Enums
    private enum ArmState {
        TRANSFER(0),
        IDLE(150),
        SUBMARINE(450),
        INTAKE(580),
        CALIBRATING(-1),
        VERIFICATION(-1);

        public final int targetPosition;

        ArmState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }
    public enum LiftState {
        GROUND(0),
        SPECIMENPICKUP(1000), // Example position - adjust as needed
        SPECIMENTRANSFER(2000), // Example position - adjust as needed
        SAMPLETRANSFER(4500);

        public final int targetPosition;

        LiftState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }
    // #endregion Enums


    // #region Drivetrain Variables
    private double DRIVETRAIN_MOTOR_SCALE = 1.0; // motor power is multiplied by this
    private double drivetrainTurboSpeedMultiplier = 1.0;
    private double drivetrainTurboSpeedTurningMultiplier = 1.0;
    private double drivetrainNormalSpeedMultiplier = 0.5;
    private double drivetrainNormalSpeedTurningMultiplier = 0.5;
    private double drivetrainSlowSpeedMultiplier = 0.5;
    private double drivetrainSlowSpeedTurningMultiplier = 0.25;
    // #endregion Drivetrain Variables


    // #region Arm Variables
    private static final double arm_kP = 0.05;
    private static final double arm_kI = 0.0000;
    private static final double arm_kD = 0.0;
    private static final double arm_kG = 0;
    private static double armMotorPower = 0.5; // Maximum power for the arm motor
    private static final double armMotorPowerHigh = 0.5;
    private static final double armMotorPowerLow = 0.3;
    private static final double armMotorPowerSuperLow = 0.1;
    private static final double ARM_START_ANGLE = 40.0; // Starting angle in degrees
    private static final double ARM_END_ANGLE = 180.0; // End angle in degrees
    private static final double ARM_ENCODER_RANGE = 530.0; // Total encoder ticks
    private static final double DEGREES_PER_TICK = (ARM_END_ANGLE - ARM_START_ANGLE) / ARM_ENCODER_RANGE;
    private double previousError = 0.0;  // Class member to store previous error
    private double errorSum = 0.0;       // Class member to store integrated error
    private static final double MAX_ERROR_SUM = 100.0; // Prevent integral windup
    private boolean isCalibrating = false;
    private ElapsedTime calibrationTimer = new ElapsedTime();
    private static final double CALIBRATION_WAIT_TIME = 0.25; // Time in seconds to wait after stopping motor
    private int errorHistoryCount = 2;
    private double[] errorHistory = new double[errorHistoryCount];
    private int errorIndex = 0;

    private static final int FF_ZONE1_START = 0;
//    private static final int FF_ZONE1_END = 136;
    private static final int FF_ZONE2_START = 190;
//    private static final int FF_ZONE2_END = 380;
    private static final int FF_ZONE3_START = 380;
//    private static final int FF_ZONE3_END = 450;
    private static final int FF_ZONE4_START = 450;
    private static final int FF_ZONE4_END = 600;
    private static final double FF_ZONE1_POWER_START = 0.8;
    private static final double FF_ZONE1_POWER_END = 0.10;
    private static final double FF_ZONE2_POWER_START = 0.1;
    private static final double FF_ZONE2_POWER_END = 0.2;
    private static final double FF_ZONE3_POWER_START = 0.2;
    private static final double FF_ZONE3_POWER_END = 0.1;
    private static final double FF_ZONE4_POWER_START = 0;
    private static final double FF_ZONE4_POWER_END = -0.1;

    private static final int FF_ZONE1_END = 160;  	// End of zero power zone
    private static final int FF_ZONE2_END = 450;  	// End of increasing power zone
    private static final int FF_ZONE3_END = 560;  	// End of decreasing power zone
    private static final double FF_MAX_POWER = 0.2;   // Max feedforward power
    private static final double FF_ZONE1_MAX_POWER = 0.1; // Adjust starting zone power





    //#endregion Arm Variables


    // #region Lift Variables
    private static final double lift_kP = 0.002;
    private static final double lift_kI = 0;
    private static final double lift_kD = 0.00;
    private static final double lift_kG = 0;
    private static final double liftMotorPower = 1; // Maximum power for the lift motor
    private static final double liftMotorPowerDown = 0.4;
    private static final int liftMinimumEncoderPosition = 0; // Joystick lower limit
    private static final int liftMaximumEncoderPosition = 6000; // Joystick upper limit
    private static final int liftTopStageEncoderPosition = 4300;
    // #endregion Lift Variables


    // #region Servo Variables
    private static final double dustpanServoPositionIntaking = 145 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double dustpanServoPositionDumping = 50.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
    private static final double specimenServoPositionClosed  = 120 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double specimenServoPositionOpen  = 50.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
    // #endregion Servo Variables


    // #region Other Variables
    private static final double intakeMotorPower = 1; // Power for the spinner motor
    private static final int POSITION_DEADBAND = 0; // do not change from 0 or the robot will act like it drank 10 cups of coffee
    private static final double INTEGRAL_LIMIT = armMotorPower; // Match to max power
    // #endregion Other Variables
    // #endregion Editable Variables
    // #endregion Variables


    // #region Main Java Loop
    @Override
    public void runOpMode() {
        initializeRobot();


        waitForStart();
        runtime.reset();
        pidTimer.reset();


        while (opModeIsActive()) {
            updateRobotState();
            stateMachineActions();
            mecanumDrivetrain();
            doMovement();
            telemetryUpdate();
            //telemetryZonedFeedForward();
        }
    }
    // #endregion Main Java Loop


    // #region Initialization Functions
    private void initializeRobot() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        linearVerticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        intakeArm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armRoller = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        dustpanServo = hardwareMap.get(Servo.class, "bucket");
        specimenIntake = hardwareMap.get(Servo.class, "claw");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


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


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    private void configureMotor(DcMotorEx motor) { // Helper function to configure motor encoder settings
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Do not use encoder for feedback


    }
    // #endregion Initialization Functions


    // #region State Machine
    // Handles movement between robot states
    private void updateRobotState() {
        // ... (Calibration logic remains the same)

        // Gamepad 2 Controls for Arm State
        if (gamepad2.a && !lastAButton) { currentArmState = ArmState.INTAKE; }
        if (gamepad2.b && !lastBButton) { currentArmState = ArmState.SUBMARINE;}
        if (gamepad2.x && !lastXButton) { currentArmState = ArmState.TRANSFER;}
        if (gamepad2.y && !lastYButton) { currentArmState = ArmState.IDLE; }

        lastAButton = gamepad2.a;
        lastBButton = gamepad2.b;
        lastXButton = gamepad2.x;
        lastYButton = gamepad2.y;

        // Gamepad 2 Controls for Lift State
        if (gamepad2.dpad_up && !lastDpadUp) { currentLiftState = LiftState.SAMPLETRANSFER; if (currentArmState == ArmState.TRANSFER) {currentArmState = ArmState.IDLE;}}
        if (gamepad2.dpad_down && !lastDpadDown) { currentLiftState = LiftState.GROUND; if (currentArmState == ArmState.TRANSFER) {currentArmState = ArmState.IDLE;}}  // Add down for Ground
        if (gamepad2.dpad_left && !lastDpadLeft) { currentLiftState = LiftState.SPECIMENPICKUP; if (currentArmState == ArmState.TRANSFER) {currentArmState = ArmState.IDLE;}}  // Add left for specimen pickup
        if (gamepad2.dpad_right && !lastDpadRight) { currentLiftState = LiftState.SPECIMENTRANSFER; if (currentArmState == ArmState.TRANSFER) {currentArmState = ArmState.IDLE;}} // Add right for specimen transfer

        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;



        // Add this logic for specimen servo toggle on start button press
        if (gamepad1.start && !lastStartButton) { // Assuming you have a lastStartButton variable
            specimenState = !specimenState;
            lastStartButton = true;
        } else if (!gamepad1.start) {
            lastStartButton = false;
        }

        // Calibration Initiation
        if (gamepad2.back && !lastBackButton) {
            if (currentArmState != ArmState.CALIBRATING) {
                currentArmState = ArmState.CALIBRATING; // Start calibration
                isCalibrating = true; // Set calibration flag
                calibrationTimer.reset(); // Reset timer for verification
            } else {
                // Optionally, allow canceling calibration by pressing back again
                currentArmState = ArmState.IDLE;
                isCalibrating = false;
            }
        }
        lastBackButton = gamepad2.back;

        // Transition from CALIBRATING to VERIFICATION when button released
        if (currentArmState == ArmState.CALIBRATING && !gamepad2.back) {
            currentArmState = ArmState.VERIFICATION;
            intakeArm.setPower(0); // Stop the motor
            calibrationTimer.reset(); // Start the verification timer
        }


        // Transition from VERIFICATION to IDLE after delay
        if (currentArmState == ArmState.VERIFICATION && calibrationTimer.seconds() >= CALIBRATION_WAIT_TIME) {
            currentArmState = ArmState.IDLE;
            isCalibrating = false; // End calibration
            intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder after calibration
            intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



    }



    private void stateMachineActions() {
        switch (currentArmState) {
            case INTAKE:
                spinnerState = gamepad2.left_bumper ? -1 : (gamepad2.left_trigger > 0.1 ? gamepad2.left_trigger * Math.abs(gamepad2.left_trigger) : 0);
                break;
            case TRANSFER:
                spinnerState = gamepad2.left_bumper ? 0.8 : (gamepad2.left_trigger > 0.1 ? -gamepad2.left_trigger * Math.abs(gamepad2.left_trigger) * 0.7 : 0);
                break;
            default:  // IDLE and SUBMARINE
                spinnerState = 0;
        }

        // Dustpan Control (Only in SAMPLETRANSFER state)
        dustpanState = ((currentLiftState == LiftState.SAMPLETRANSFER || currentLiftState == LiftState.SPECIMENTRANSFER) && gamepad2.left_trigger > 0.2);


        // Specimen Control (Only in SPECIMENPICKUP and SPECIMENTRANSFER) disabled for now
//        if (currentLiftState == LiftState.SPECIMENPICKUP || currentLiftState == LiftState.SPECIMENTRANSFER) {
//            if (gamepad1.dpad_up && !lastDpadUp) { //added gamepad 1 control
//                specimenState = !specimenState;
//            }
//            lastDpadUp = gamepad1.dpad_up;
//        }

    }



    private void doMovement() {
        // ... (Servo logic remains the same - use dustpanState and specimenState)

        armRoller.setPower(spinnerState * intakeMotorPower);

        if (!isCalibrating) { // Check for calibration before running PID
            if (armPIDActive) {
                armCurrentEncoderPosition = intakeArm.getCurrentPosition();
                armPower = calculateArmPIDG(); // Or your preferred PID method
                // ... (Limit arm power as before) ...
                intakeArm.setPower(armPower);
            }

            if (slidePIDActive) {
                liftCurrentEncoderPosition = linearVerticalSlide.getCurrentPosition();
                slidePower = calculateSlideP(); // Or your preferred PID method
                limitedSlidePower = getLimitedPower(slidePower, (int) liftCurrentEncoderPosition, currentLiftState.targetPosition, currentLiftState.targetPosition);
                linearVerticalSlide.setPower(limitedSlidePower);
            }
        } else if (currentArmState == ArmState.CALIBRATING) {
            intakeArm.setPower(-0.3); // Apply power for calibration
        } else {
            intakeArm.setPower(0); // Stop the motor during verification
        }

        if (dustpanState) {
            dustpanServo.setPosition(dustpanServoPositionDumping); // Position 2 for dumping
        } else {
            dustpanServo.setPosition(dustpanServoPositionIntaking); // Position 1 for intaking
        }

        if (specimenState) {
            specimenIntake.setPosition(specimenServoPositionOpen);
        } else {
            specimenIntake.setPosition(specimenServoPositionClosed);
        }
    }

    // #endregion State Machine


    // #region Helper Functions


    private void mecanumDrivetrain() { // Handles controller inputs and setting drivetrain power (almost polished)


        reducedSpeed = (gamepad1.right_bumper || gamepad2.right_bumper); // Check for right bumper pressed
        turboMode = (gamepad1.right_trigger > 0.2); // Check if the right trigger is pressed

        reducedSpeed = (currentLiftState == LiftState.SAMPLETRANSFER);
        if (reducedSpeed) {
            speedMultiplier = drivetrainSlowSpeedMultiplier;
        } else if (turboMode) {
            speedMultiplier = drivetrainTurboSpeedMultiplier;
        } else {
            speedMultiplier = drivetrainNormalSpeedMultiplier;
        }


        // Driving controls (Gamepad 1) with scaling and speed adjustments
        axial = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * speedMultiplier;
        lateral = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * speedMultiplier;
        yaw = -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * speedMultiplier;


        if (reducedSpeed) { // Apply cubic scaling if reduced speed is active
            axial = axial *  speedMultiplier; //add the * speedmultiplier if you want it to be even slower
            lateral = lateral *  speedMultiplier;
            yaw = yaw * speedMultiplier;
        }


        // Calculate individual wheel powers based on driver input. Using helper function
        powers = calculateWheelPowers(axial, lateral, yaw);


        // Set drive motor powers.
        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftBackDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }


    private double calculateZoneFF(int position) {
        double linearFF = 0;

        // Zone 1: 0-136
        if (position >= FF_ZONE1_START && position <= FF_ZONE1_END) {
            double progress = (position - FF_ZONE1_START) / (double)(FF_ZONE1_END - FF_ZONE1_START);
            linearFF = FF_ZONE1_POWER_START + (FF_ZONE1_POWER_END - FF_ZONE1_POWER_START) * progress;
        }
        // Zone 2: 190-380
        else if (position >= FF_ZONE2_START && position <= FF_ZONE2_END) {
            double progress = (position - FF_ZONE2_START) / (double)(FF_ZONE2_END - FF_ZONE2_START);
            linearFF = FF_ZONE2_POWER_START + (FF_ZONE2_POWER_END - FF_ZONE2_POWER_START) * progress;
        }
        // Zone 3: 380-450
        else if (position >= FF_ZONE3_START && position <= FF_ZONE3_END) {
            double progress = (position - FF_ZONE3_START) / (double)(FF_ZONE3_END - FF_ZONE3_START);
            linearFF = FF_ZONE3_POWER_START + (FF_ZONE3_POWER_END - FF_ZONE3_POWER_START) * progress;
        }
        // Zone 4: 450-600
        else if (position >= FF_ZONE4_START && position <= FF_ZONE4_END) {
            double progress = (position - FF_ZONE4_START) / (double)(FF_ZONE4_END - FF_ZONE4_START);
            linearFF = FF_ZONE4_POWER_START + (FF_ZONE4_POWER_END - FF_ZONE4_POWER_START) * progress;
        }
        // Handle gap between zones 1 and 2 (136-190)
        else if (position > FF_ZONE1_END && position < FF_ZONE2_START) {
            double progress = (position - FF_ZONE1_END) / (double)(FF_ZONE2_START - FF_ZONE1_END);
            linearFF = FF_ZONE1_POWER_END + (FF_ZONE2_POWER_START - FF_ZONE1_POWER_END) * progress;
        }
        // Outside any zone - explicitly return 0
        else {
            linearFF = 0;
        }

        return linearFF;
    }



    private double calculateArmPIDG() {
        double error = currentArmState.targetPosition - armCurrentEncoderPosition;



        if (Math.abs(error) <= POSITION_DEADBAND) {
            return 0.0;
        }



        double deltaTime = pidTimer.seconds();
        pidTimer.reset();



        double P = error * arm_kP;



        armIntegral += error * deltaTime;
        armIntegral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, armIntegral));
        double I = armIntegral * arm_kI;



        double derivative = (error - lastArmError) / deltaTime;
        derivative *= 0.7; // Damping if necessary
        double D = derivative * arm_kD;


        lastArmError = error;


        double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition * DEGREES_PER_TICK);
        double angleRadians = Math.toRadians(currentAngle);
        double feedforward = arm_kG * Math.cos(angleRadians);  // Gravity compensation


        double linearFF = 0;
        if (armCurrentEncoderPosition <= FF_ZONE1_END) {
            double zoneProgress = armCurrentEncoderPosition / (double) FF_ZONE1_END;
            linearFF = zoneProgress * FF_ZONE1_MAX_POWER;
        } else if (armCurrentEncoderPosition <= FF_ZONE2_END) {
            double zoneProgress = (armCurrentEncoderPosition - FF_ZONE1_END) / (double) (FF_ZONE2_END - FF_ZONE1_END);
            linearFF = FF_ZONE1_MAX_POWER + (zoneProgress * (FF_MAX_POWER - FF_ZONE1_MAX_POWER));
        } else if (armCurrentEncoderPosition <= FF_ZONE3_END) {
            double zoneProgress = (armCurrentEncoderPosition - FF_ZONE2_END) / (double) (FF_ZONE3_END - FF_ZONE2_END);
            linearFF = FF_MAX_POWER * (1 - zoneProgress);
        }


        double output = P + I + D + feedforward + linearFF;



//        if (currentArmState == ArmState.TRANSFER && armCurrentEncoderPosition < 400 && armCurrentEncoderPosition >= 155) {
//            armMotorPower = armMotorPowerSuperLow;
//        } else if (armCurrentEncoderPosition < 155 && (currentArmState == ArmState.TRANSFER) ){
//            armMotorPower = armMotorPowerLow; // Use the higher power by default
//        } else {
//            armMotorPower = armMotorPowerHigh; // Use the higher power by default
//        }

        if (output > armMotorPower) {
            output = armMotorPower;
        } else if (output < -armMotorPower) {
            output = -armMotorPower;
        }



        return output;
    }


    private double calculateArmPIDGold() {
        // Calculate current error
        double error = currentArmState.targetPosition - armCurrentEncoderPosition;
        errorHistory[errorIndex] = error;
        errorIndex = (errorIndex + 1) % errorHistoryCount;
        double averageError = calculateAverageError();



        // Check if within acceptable error range
        if (Math.abs(error) <= POSITION_DEADBAND) {
            // Reset integral term when target is reached
            error = 0;
            previousError = error;
        }


        // Calculate P term
        double P = error * arm_kP;


        // Calculate I term with anti-windup
        errorSum += error;
        // Clamp integral term to prevent windup
        errorSum = Math.min(Math.max(errorSum, -MAX_ERROR_SUM), MAX_ERROR_SUM);
        double I = errorSum * arm_kI;


        // Calculate D term using change in error
        double errorChange = error - previousError;
        double D = errorChange * arm_kD;


        // Calculate gravity compensation
        double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition * DEGREES_PER_TICK);
        double angleRadians = Math.toRadians(currentAngle);
        double feedforward = arm_kG * Math.cos(angleRadians);

        // Calculate zoned feed forward
        double linearFF = calculateZoneFF((int) armCurrentEncoderPosition);

        // Combine P, I, D, and G terms
        double output = P + I + D + feedforward + linearFF;


        // Limit output to motor power constraints
        if (output > armMotorPower) {
            output = armMotorPower;
            // Anti-windup: prevent integral from growing when output is saturated
            if (error > 0) errorSum -= error;
        } else if (output < -armMotorPower) {
            output = -armMotorPower;
            // Anti-windup: prevent integral from growing when output is saturated
            if (error < 0) errorSum -= error;
        }


        // Store current error for next iteration
        previousError = error;


        return output;
    } //v4





    private double calculateSlideP() {
        // Calculate position error
        double error = currentLiftState.targetPosition - liftCurrentEncoderPosition;
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




    private double[] calculateWheelPowers(double axial, double lateral, double yaw) {
        double[] powers = new double[4];


        powers[0] = (axial - lateral + yaw) * DRIVETRAIN_MOTOR_SCALE; // Left Front
        powers[1] = (axial + lateral - yaw) * DRIVETRAIN_MOTOR_SCALE; // Right Front
        powers[2] = (axial + lateral + yaw) * DRIVETRAIN_MOTOR_SCALE; // Left Back
        powers[3] = (axial - lateral - yaw) * DRIVETRAIN_MOTOR_SCALE; // Right Back


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


    private void telemetryUpdate() {
        // Drivetrain Information
//        telemetry.addData("Axial Input", axial);
//        telemetry.addData("Lateral Input", lateral);
//        telemetry.addData("Yaw Input", yaw);
//        telemetry.addData("Speed Multiplier", speedMultiplier);
//        telemetry.addData("Left Front Power", leftFrontDrive.getPower());
//        telemetry.addData("Right Front Power", rightFrontDrive.getPower());
//        telemetry.addData("Left Back Power", leftBackDrive.getPower());
//        telemetry.addData("Right Back Power", rightBackDrive.getPower());

        // Arm and Lift Information
        telemetry.addData("Current Arm State", currentArmState);
        telemetry.addData("Arm Target Position", currentArmState.targetPosition);
        telemetry.addData("Arm Current Position", intakeArm.getCurrentPosition());
        telemetry.addData("Arm Power (Calculated)", armPower);  // Add limitedArmPower if used
        telemetry.addData("Current Lift State", currentLiftState);
        telemetry.addData("Lift Target Position", currentLiftState.targetPosition);
        telemetry.addData("Lift Current Position", linearVerticalSlide.getCurrentPosition());
        telemetry.addData("Lift Power (Calculated)", slidePower); // Add limitedSlidePower if used

        // Intake and Dustpan
        telemetry.addData("Dustpan State", dustpanState);
        telemetry.addData("Spinner State", spinnerState);

        // Other Debugging Information (as needed)
        telemetry.addData("Limits Enabled", limitsEnabled);


        // ... any other variables you want to monitor

        // PID Debugging Information
        double avgError = calculateAverageError();
        telemetry.addData("Average Error (Arm)", avgError);


        // Display individual error values (Formatted for readability)
        StringBuilder errorValues = new StringBuilder("Error History: ");
        for (int i = 0; i < errorHistory.length; i++) {
            errorValues.append(String.format("%.2f", errorHistory[i])).append(", "); // Format to 2 decimal places
        }
        telemetry.addData(errorValues.toString().substring(0, errorValues.length()-2), ""); // Remove trailing comma and space


        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryZonedFeedForward() {
        int currentPos = (int) armCurrentEncoderPosition;
        double currentFF = calculateZoneFF(currentPos);

        String zone;
        if (currentPos >= FF_ZONE1_START && currentPos <= FF_ZONE1_END) {
            zone = "Zone 1";
        } else if (currentPos > FF_ZONE1_END && currentPos < FF_ZONE2_START) {
            zone = "Transition 1-2";
        } else if (currentPos >= FF_ZONE2_START && currentPos <= FF_ZONE2_END) {
            zone = "Zone 2";
        } else if (currentPos >= FF_ZONE3_START && currentPos <= FF_ZONE3_END) {
            zone = "Zone 3";
        } else if (currentPos >= FF_ZONE4_START && currentPos <= FF_ZONE4_END) {
            zone = "Zone 4";
        } else {
            zone = "Outside Zones";
        }

        telemetry.addData("Encoder Position", currentPos);
        telemetry.addData("Current Zone", zone);
        telemetry.addData("Current FF Power", currentFF);
        telemetry.update();
    }









    // #endregion Helper Functions
    private double calculateAverageError() {
        double sum = 0;
        int count = 0; // Keep track of valid entries
        for (double err : errorHistory) {
            if (!Double.isNaN(err)) { // Check for NaN values
                sum += err;
                count++;
            }
        }
        return count > 0 ? sum / count : 0; // Handle case where no valid entries exist
    }



}