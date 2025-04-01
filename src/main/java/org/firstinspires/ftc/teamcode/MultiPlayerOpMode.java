package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




    @TeleOp(name = "B-V3 TWO Player", group = "Linear OpMode")
    public class MultiPlayerOpMode extends LinearOpMode {




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
        private boolean armPIDActive = true; // Flag to enable/disable PID control for the arm
        private boolean slidePIDActive = true; // Flag to enable/disable PID control for the slide
        private double spinnerState = 0; // State of the spinner (0: off, 1: forward, -1: reverse)
        private ElapsedTime pidTimer = new ElapsedTime(); // Timer for PID loop
        private double lastArmError = 0; // Previous error for arm PID
        private double lastSlideError = 0; // Previous error for slide PID
        private double armIntegral = 0; // Accumulated error for arm PID
        private double slideIntegral = 0; // Accumulated error for slide PID
        private int currentArmTarget = 0; // Current target position for the arm
        private int currentSlideTarget = 0; // Current target position for the slide
        private ElapsedTime servoToggleTimer = new ElapsedTime();
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
        private RobotState currentState = RobotState.IDLE;
        private RobotState previousState = RobotState.IDLE;
        private boolean dpad_right_pressed = false;
        private boolean dpad_left_pressed = false;
        private double liftCurrentEncoderPosition = 0;
        private double armCurrentEncoderPosition = 0;
        private double liftPIDValue;




        //private RobotState currentRobotState = RobotState.IDLE;
        private boolean lastDpadUp = false;
        private boolean lastDpadDown = false;
        private boolean firstArmMotorCalibrate = true;




        // #endregion Uneditable Variables




        // #region Editable Variables




        // #region Enums
        //
        enum RobotState {
            // Robot intended gameplay path (must be able to reverse)
            // IDLE -> SUBMARINE -> INTAKEPOSITION1 -> INTAKING -> INTAKEPOSITION2 -> SUBMARINE2 -> TRANSFERPOSITION -> TRANSFERING -> IDLE2 -> LIFTED -> DUMPING -> LIFTED2 -> IDLE ...
            IDLE, // robot in parked state for travelling
            IDLEMOVEMENT,
            SUBMARINE, // arm in middle position to enter submarine
            INTAKEPOSITION1, // first intake position before intaking
            INTAKING, // spinner is intaking
            INTAKEPOSITION2, // second intake position after intaking
            SUBMARINE2, // second submarine position after intake
            IDLE3, // idle before depositing into bucket
            TRANSFERPOSITION, // position to position the intake to push into the bucket
            TRANSFERING, // spin intake reverse to put in bucket
            IDLE2, // second idle state after transfering
            LIFTED, // first lifted state before dumping
            DUMPING, // dumping state
            LIFTED2, // second lifted state after dumping




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
        private static final double arm_kP = 0.01;
        private static final double arm_kI = 0.0000;
        private static final double arm_kD = 0.00;
        private static final double arm_kG = 0;
        private static final int FF_ZONE1_END = 160;      // End of zero power zone
        private static final int FF_ZONE2_END = 450;      // End of increasing power zone
        private static final int FF_ZONE3_END = 560;      // End of decreasing power zone
        private static final double FF_MAX_POWER = 0.2;
        private static final double FF_ZONE1_MAX_POWER = 0.1;
        private static final int armTargetPositionIntaking = 600; // position for intaking
        private static final int armTargetPositionWhileEnteringOrExitingSubmarine = 450; // position for entering and exiting submarine
        private static final int armTargetPositionParking = 150; // position for parking to raise lift
        private static final int armTargetPositionWhileDepositingIntoTheDustPan = 0; // position for depositing sample into box
        private static final double armMotorPower = 0.4; // Maximum power for the arm motor
        private static final double ARM_START_ANGLE = 40.0; // Starting angle in degrees
        private static final double ARM_END_ANGLE = 180.0; // End angle in degrees
        private static final double ARM_ENCODER_RANGE = 530.0; // Total encoder ticks
        private static final double DEGREES_PER_TICK = (ARM_END_ANGLE - ARM_START_ANGLE) / ARM_ENCODER_RANGE;
        private double previousError = 0.0;  // Class member to store previous error
        private double errorSum = 0.0;       // Class member to store integrated error
        private static final double MAX_ERROR_SUM = 100.0; // Prevent integral windup
        private boolean isCalibrating = false;
        private ElapsedTime calibrationTimer = new ElapsedTime();
        private static final double CALIBRATION_WAIT_TIME = 0.2; // Time in seconds to wait after stopping motor








        //#endregion Arm Variables




        // #region Lift Variables
        private static final double lift_kP = 0.001;
        private static final double lift_kI = 0;
        private static final double lift_kD = 0.00;
        private static final double lift_kG = 0;
        private static final double liftMotorPower = 1; // Maximum power for the lift motor
        private static final int liftMinimumEncoderPosition = 0; // Joystick lower limit
        private static final int liftMaximumEncoderPosition = 6000; // Joystick upper limit
        private static final int liftTopStageEncoderPosition = 4300;
        // #endregion Lift Variables




        // #region Servo Variables
        private static final double dustpanServoPositionIntaking = 50 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
        private static final double dustpanServoPositionDumping = 145.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
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
                // telemetryUpdate(); // disabled when not debugging to decrease loop time (only use one at a time)
                //telemetryUpdateVerbose(); // disabled when not debugging to decrease loop time (only use one telemetry update at a time)
                telemetryPID();
                //telemetry.addData("Encoder Position", intakeArm.getCurrentPosition());
                //telemetry.update();
                manualPID();




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
            dustpanServo.setPosition(dustpanServoPositionDumping);
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
            previousState = currentState;
            if (gamepad2.back) {
                if (!lastBackButton) {
                    // Button just pressed - store current state
                    //previousState = currentState;
                    armPIDActive = false;  // Disable PID when manual override starts
                    isCalibrating = true;




                }
                // Apply manual negative power while held
                //intakeArm.setPower(-0.3);  // You can adjust this power value as needed
            } else if (lastBackButton) {
                // Button just released




                armPIDActive = true;  // Re-enable PID
                calibrationTimer.reset();
                // currentState = previousState;  // Restore previous state
            }
            lastBackButton = gamepad2.back;




            if (isCalibrating && !gamepad2.back && calibrationTimer.seconds() >= CALIBRATION_WAIT_TIME) {
                // Enough time has passed after releasing button, now safe to reset encoder
                configureMotor(intakeArm);
                armPIDActive = true;
                isCalibrating = false;
            }




            // Right DPAD debounce












            if (gamepad2.dpad_right && !dpad_right_pressed) {
                dpad_right_pressed = true;
                switch (currentState) {
                    case IDLE:
                        currentState = RobotState.SUBMARINE;
                        break;
                    case SUBMARINE:
                        currentState = RobotState.INTAKEPOSITION1;
                        break;
                    case INTAKEPOSITION1:
                        currentState = RobotState.SUBMARINE2;
                        break;
                    case SUBMARINE2:
                        currentState = RobotState.TRANSFERPOSITION;
                        break;
                    case TRANSFERPOSITION:
                        currentState = RobotState.IDLE2;
                        break;
                    case IDLE2:
                        currentState = RobotState.LIFTED;
                        break;
                    case LIFTED:
                        currentState = RobotState.IDLE;
                        break;
                }
            } else if (!gamepad2.dpad_right) {
                dpad_right_pressed = false;
            }
            // Left DPAD debounce
            if (gamepad2.dpad_left && !dpad_left_pressed) {
                dpad_left_pressed = true;
                switch (currentState) {
                    case SUBMARINE:
                        currentState = RobotState.IDLE;
                        break;
                    case INTAKEPOSITION1:
                        currentState = RobotState.SUBMARINE;
                        break;
                    case SUBMARINE2:
                        currentState = RobotState.INTAKEPOSITION1;
                        break;
                    case TRANSFERPOSITION:
                        currentState = RobotState.SUBMARINE2;
                        break;
                    case IDLE2:
                        currentState = RobotState.TRANSFERPOSITION;
                        break;
                    case LIFTED:
                        currentState = RobotState.IDLE2;
                        break;
                    case DUMPING:
                        currentState = RobotState.LIFTED;
                        break;
                    case LIFTED2:
                        currentState = RobotState.DUMPING;
                        break;
                    case IDLE:
                        currentState = RobotState.LIFTED;
                        break;
                }
            } else if (!gamepad2.dpad_left) {
                dpad_left_pressed = false;
            }
        }




        private void stateMachineActions() {




            int originalArmTarget = currentArmTarget;
            int originalSlideTarget = currentSlideTarget;




            switch (currentState) {
                case IDLE:
                    configureState(armTargetPositionParking, true, liftMinimumEncoderPosition, true, 0, false, true);
                    break;




                case SUBMARINE:
                    configureState(armTargetPositionWhileEnteringOrExitingSubmarine, true, liftMinimumEncoderPosition, true, 0, false, true);
                    break;




                case INTAKEPOSITION1:
                    if (gamepad2.left_bumper) {
                        configureState(armTargetPositionIntaking, true, liftMinimumEncoderPosition, true, 1, false, true); // Spinner on
                    } else if (gamepad2.left_trigger > 0.1 ){
                        configureState(armTargetPositionIntaking, true, liftMinimumEncoderPosition, true, -gamepad2.left_trigger*Math.abs(gamepad2.left_trigger), false, true); // Spinner on
                    } else if (gamepad2.left_trigger < 0.1 && !gamepad2.left_bumper){
                        configureState(armTargetPositionIntaking, true, liftMinimumEncoderPosition, true, 0, false, true); // Spinner on
                    } else {
                        configureState(armTargetPositionIntaking, true, liftMinimumEncoderPosition, true, 0, false, true); // Spinner on
                    }
                    break;




                case SUBMARINE2:
                    configureState(armTargetPositionWhileEnteringOrExitingSubmarine, true, liftMinimumEncoderPosition, true, 0, false, true);
                    break;




                case TRANSFERPOSITION:
                    if (gamepad2.left_bumper) {
                        configureState(armTargetPositionWhileDepositingIntoTheDustPan, true, liftMinimumEncoderPosition, true, -0.8, false, true);
                    }  else if (gamepad2.left_trigger < 0.1 && !gamepad2.left_bumper) {
                        configureState(armTargetPositionWhileDepositingIntoTheDustPan, true, liftMinimumEncoderPosition, true, 0, false, true);
                    } else if (gamepad2.left_trigger > 0.1) {
                        configureState(armTargetPositionWhileDepositingIntoTheDustPan, true, liftMinimumEncoderPosition, true, gamepad2.left_trigger*Math.abs(gamepad2.left_trigger), false, true);
                    } else {
                        configureState(armTargetPositionWhileDepositingIntoTheDustPan, true, liftMinimumEncoderPosition, true, 0, false, true);
                    }
                    firstArmMotorCalibrate = true;
                    break;




                case TRANSFERING:
                    configureState(armTargetPositionWhileDepositingIntoTheDustPan, true, liftMinimumEncoderPosition, true,0.4, false, true); // Reverse spinner




                    break;




                case IDLE2:
                    if (firstArmMotorCalibrate) {
                        firstArmMotorCalibrate = false;
                        configureMotor(intakeArm); // reset encoder pos
                    }




                    configureState(armTargetPositionParking, true, liftMinimumEncoderPosition, true, 0, false, true);
                    break;




                case LIFTED:
                    configureState(armTargetPositionParking, true, liftTopStageEncoderPosition, true, 0, gamepad2.left_trigger > 0.2, true); // Lift raised
                    break;
            }
        }




        private void doMovement() {
            // does all movement for motors and servos




            // Apply servo positions based on states
            dustpanServo.setPosition(dustpanState ? dustpanServoPositionIntaking : dustpanServoPositionDumping);








            // Manual specimen/claw control using dpad up/down
            if (gamepad2.a && !lastDpadUp) {
                specimenState = !specimenState;
                servoToggleTimer.reset();
            }
            lastDpadUp = gamepad2.a; //change from dpadup to a




            // Apply specimen/claw servo position based on state
            specimenIntake.setPosition(specimenState ? specimenServoPositionOpen : specimenServoPositionClosed);








            if (armRoller.getPower() > spinnerState*intakeMotorPower) {
                armRoller.setPower(spinnerState*intakeMotorPower);
            }
            armRoller.setPower(spinnerState * intakeMotorPower);




            // Run PID control for arm and slide if active




            if (isCalibrating && gamepad2.back) {
                // Continue applying calibration power while in calibration mode and button held
                intakeArm.setPower(-0.3);
            } else if (isCalibrating) {
                // Manual override mode
                intakeArm.setPower(0);
            } else if (armPIDActive) {




                armCurrentEncoderPosition = intakeArm.getCurrentPosition();




                armPower = calculateArmPIDG();












                //limitedArmPower = getLimitedPower(armPower, (int) armCurrentEncoderPosition, armMinimumEncoderPosition, armMaximumEncoderPosition);




                if (armPower > armMotorPower) {
                    armPower = armMotorPower;
                } else if (armPower < -armMotorPower) {
                    armPower = -armMotorPower;
                }
                intakeArm.setPower(armPower);
            }




            if (slidePIDActive) {




                liftCurrentEncoderPosition = linearVerticalSlide.getCurrentPosition();




                slidePower = calculateSlideP();




                limitedSlidePower = getLimitedPower(slidePower, (int) liftCurrentEncoderPosition, liftMinimumEncoderPosition, liftMaximumEncoderPosition);




                linearVerticalSlide.setPower(limitedSlidePower);




            }




        }
        // #endregion State Machine




        // #region Helper Functions




        private void mecanumDrivetrain() { // Handles controller inputs and setting drivetrain power (almost polished)




            reducedSpeed = (gamepad1.right_bumper || gamepad2.right_bumper); // Check for right bumper pressed
            turboMode = (gamepad1.right_trigger > 0.2); // Check if the right trigger is pressed




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
                double currentAngle = ARM_START_ANGLE + (currentPosition * DEGREES_PER_TICK);
                double angleRadians = Math.toRadians(currentAngle);
                double feedforward = targetPosition * Math.cos(angleRadians);
                ARM_PID_OUTPUT = P + I + D; //store for telemetry
                ARM_PID_FF_VALUE = P + I + D + feedforward;
                return P + I + D + feedforward;
            } else {
                lastSlideError = error;
                return P + I + D;
            }
        } //the og




        private double calculateArmPID() {




            double error = currentArmTarget - armCurrentEncoderPosition;




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




            derivative *= 0.7; // Scaling derivative if needed




            double D = derivative * arm_kD;




            lastArmError = error;




            //double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition * DEGREES_PER_TICK);
            double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition *  DEGREES_PER_TICK);




            double angleRadians = Math.toRadians(currentAngle);




            double feedforward = arm_kG * Math.cos(angleRadians);




            ARM_PID_OUTPUT = P + I + D;
            if (ARM_PID_OUTPUT > armMotorPower) {
                ARM_PID_OUTPUT = armMotorPower;
            } else if (ARM_PID_OUTPUT < -armMotorPower) {
                ARM_PID_OUTPUT = -armMotorPower;
            }




            ARM_PID_FF_VALUE = P + I + D + feedforward;
            if (ARM_PID_FF_VALUE > armMotorPower) {
                ARM_PID_FF_VALUE = armMotorPower;
            } else if (ARM_PID_FF_VALUE < -armMotorPower) {
                ARM_PID_FF_VALUE = -armMotorPower;
            }




            return P;




        } // original




        private double calculateArmPG() {
            // Calculate error
            double error = currentArmTarget - armCurrentEncoderPosition;




            // Check if within acceptable error range
            if (Math.abs(error) <= POSITION_DEADBAND) {
                return 0.0;
            }




            // Calculate P term
            double P = error * arm_kP;




            // Calculate gravity compensation
            double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition * DEGREES_PER_TICK);
            double angleRadians = Math.toRadians(currentAngle);
            double feedforward = arm_kG * Math.cos(angleRadians);




            // Combine P and G terms
            double output = P + feedforward;




            // Limit output to motor power constraints
            if (output > armMotorPower) {
                output = armMotorPower;
            } else if (output < -armMotorPower) {
                output = -armMotorPower;
            }




            return output;
        } //v2




        private double calculateArmPDG() {
            // Calculate current error
            double error = currentArmTarget - armCurrentEncoderPosition;




            // Check if within acceptable error range
            if (Math.abs(error) <= POSITION_DEADBAND) {
                previousError = error;  // Update previous error before returning
                return 0.0;
            }




            // Calculate P term
            double P = error * arm_kP;




            // Calculate D term using change in error
            double errorChange = error - previousError;
            double D = errorChange * arm_kD;




            // Calculate gravity compensation
            double currentAngle = ARM_START_ANGLE + (armCurrentEncoderPosition * DEGREES_PER_TICK);
            double angleRadians = Math.toRadians(currentAngle);
            double feedforward = arm_kG * Math.cos(angleRadians);




            // Combine P, D, and G terms
            double output = P + D + feedforward;




            // Limit output to motor power constraints
            if (output > armMotorPower) {
                output = armMotorPower;
            } else if (output < -armMotorPower) {
                output = -armMotorPower;
            }




            // Store current error for next iteration
            previousError = error;




            return output;
        } // v3




        private double calculateArmPIDG() {
            // Calculate current error
            double error = currentArmTarget - armCurrentEncoderPosition;




            // Check if within acceptable error range
            if (Math.abs(error) <= POSITION_DEADBAND) {
                // Reset integral term when target is reached
                errorSum = 0.0;
                previousError = error;
                return 0.0;
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
            double linearFF = 0;
            if (armCurrentEncoderPosition <= FF_ZONE1_END) {
                // Zone 1: Linear increase from 0 to FF_ZONE1_MAX_POWER
                double zoneProgress = armCurrentEncoderPosition / (double)FF_ZONE1_END;
                linearFF = zoneProgress * FF_ZONE1_MAX_POWER;
            } else if (armCurrentEncoderPosition <= FF_ZONE2_END) {
                // Zone 2: Linear increase from ZONE1_MAX_POWER to MAX_POWER
                double zoneProgress = (armCurrentEncoderPosition - FF_ZONE1_END) /
                        (double)(FF_ZONE2_END - FF_ZONE1_END);
                linearFF = FF_ZONE1_MAX_POWER + (zoneProgress * (FF_MAX_POWER - FF_ZONE1_MAX_POWER));
            } else if (armCurrentEncoderPosition <= FF_ZONE3_END) {
                // Zone 3: Linear decrease from max power to 0
                double zoneProgress = (armCurrentEncoderPosition - FF_ZONE2_END) /
                        (double)(FF_ZONE3_END - FF_ZONE2_END);
                linearFF = FF_MAX_POWER * (1 - zoneProgress);
            }
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




        private double calculateSlidePID() {




            double error = currentSlideTarget - liftCurrentEncoderPosition;




            if (Math.abs(error) <= POSITION_DEADBAND) {




                return 0.0;




            }




            double deltaTime = pidTimer.seconds();




            pidTimer.reset();




            double P = error * lift_kP;




            slideIntegral += error * deltaTime;




            slideIntegral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, slideIntegral));




            double I = slideIntegral * lift_kI;




            double derivative = (error - lastSlideError) / deltaTime;




            double D = derivative * lift_kD;




            lastSlideError = error;




            liftPIDValue = P + I + D;
            if (liftPIDValue > liftMotorPower) {
                liftPIDValue = liftMotorPower;
            } else if (liftPIDValue < -liftMotorPower) {
                liftPIDValue = -liftMotorPower;
            }




            return P + I + D;




        }




        private double calculateSlideP() {
            // Calculate position error
            double error = currentSlideTarget - liftCurrentEncoderPosition;




            // Return 0 if within deadband to prevent small oscillations
            if (Math.abs(error) <= POSITION_DEADBAND) {
                return 0.0;
            }




            // Calculate P term
            double P = error * lift_kP;




            // Limit the output to the maximum motor power
            if (P > liftMotorPower) {
                P = liftMotorPower;
            } else if (P < -liftMotorPower) {
                P = -liftMotorPower;
            }




            return P;
        }




        private void manualPID() {
            if (gamepad2.start) {
                configureMotor(intakeArm);
            }
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




        private void telemetryUpdate() { // Handles telemetry updates (keep off when not debugging to reduce cpu loop time)
            telemetry.addData("Robot State", currentState);
            telemetry.addData("Arm Target", currentArmTarget);
            telemetry.addData("Slide Target", currentSlideTarget);
            telemetry.update();
        }




        private void telemetryUpdateVerbose() {
            telemetry.addData("LIMITS", limitsEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Arm Position", intakeArm.getCurrentPosition());
            telemetry.addData("Arm Target", currentArmTarget);
            telemetry.addData("Arm PID Active", armPIDActive);
            telemetry.addData("Arm PID FF Value", ARM_PID_FF_VALUE);
            telemetry.addData("Arm Motor Power", limitedArmPower);
            telemetry.addData("Arm Motor PID Power", ARM_PID_OUTPUT);
            telemetry.addData("Slide Position", linearVerticalSlide.getCurrentPosition());
            telemetry.addData("Slide Target", currentSlideTarget);
            telemetry.addData("Slide PID Active", slidePIDActive);
            telemetry.addData("Dustpan State", dustpanState ? "90°" : "45°");
            telemetry.addData("Specimen State", specimenState ? "90°" : "45°");
            telemetry.addData("controller position", gamepad2.left_trigger);
            telemetry.addData("spinner state", spinnerState);
            telemetry.addData("leftFrontDrivePower", powers[0]);
            telemetryUpdate();
        }




        private void telemetryPID() {
            // Arm PID telemetry
            telemetry.addData("Arm Current Position", armCurrentEncoderPosition);
            telemetry.addData("Arm Target Position", currentArmTarget);
            telemetry.addData("Arm Error", currentArmTarget - armCurrentEncoderPosition);
            telemetry.addData("Arm PID Output", ARM_PID_OUTPUT);
            telemetry.addData("Arm PID+FF Output", ARM_PID_FF_VALUE);
            telemetry.addData("Arm Final Power", armPower);
            telemetry.addData("Arm PID Active", armPIDActive);




            // Slide PID telemetry
            telemetry.addData("Slide Current Position", liftCurrentEncoderPosition);
            telemetry.addData("Slide Target Position", currentSlideTarget);
            telemetry.addData("Slide Error", currentSlideTarget - liftCurrentEncoderPosition);
            telemetry.addData("Slide PID Output", liftPIDValue);
            telemetry.addData("Slide Final Power", slidePower);
            telemetry.addData("Slide PID Active", slidePIDActive);




            telemetry.update();
        }




        private void configureState(double armPos, boolean armPIDState, double liftPos, boolean slidePIDState, double spinnerSpeed, boolean dustpanPos, boolean specimenPos) {
            if (gamepad2.b && currentState == RobotState.INTAKEPOSITION1) {
                currentArmTarget = (int) armPos + 20;  // Increase arm setpoint by 20
            } else if (gamepad2.b && (currentState == RobotState.LIFTED ||
                    currentState == RobotState.DUMPING ||
                    currentState == RobotState.LIFTED2)) {
                currentSlideTarget = (int) liftPos + 500;  // Increase lift setpoint by 200
            } else {
                currentArmTarget = (int) armPos;
                currentSlideTarget = (int) liftPos;
            }




            armPIDActive = armPIDState;
            slidePIDActive = slidePIDState;
            spinnerState = spinnerSpeed;
            dustpanState = dustpanPos;
            //specimenState = specimenPos;
        }
        // #endregion Helper Functions




    }

