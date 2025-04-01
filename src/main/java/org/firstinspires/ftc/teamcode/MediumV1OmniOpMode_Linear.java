/*package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Medium V1 Omni Linear OpMode", group = "Linear OpMode")
public class MediumV1OmniOpMode_Linear extends LinearOpMode {


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


  public Servo intake; // Servo for intake mechanism
  public Servo specimenIntake; // Servo for specimen intake
  public Servo dustpanServo; // Servo for dustpan
  // #endregion Declarations


  // #region Uneditable Variables
  private boolean dustpanState = false; // Current state of the dustpan
  private boolean specimenState = false; // Current state of the specimen intake
  private boolean lastRightBumper = false; // Previous state of the right bumper
  private boolean lastLeftBumper = false; // Previous state of the left bumper
  private boolean lastAButton = false; // Previous state of the A button
  private boolean lastBButton = false; // Previous state of the B button
  private boolean armPIDActive = true; // Flag to enable/disable PID control for the arm
  private boolean slidePIDActive = true; // Flag to enable/disable PID control for the slide
  private int spinnerState = 0; // State of the spinner (0: off, 1: forward, -1: reverse)
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


  private RobotState currentRobotState = RobotState.IDLE;
  private boolean lastDpadUp = false;
  private boolean lastDpadDown = false;
  private double armInput;

  // #endregion Uneditable Variables


  // #region Editable Variables


  // #region Enums
  //
  enum RobotState {
    // Robot intended gameplay path (must be able to reverse)
    // IDLE -> SUBMARINE -> INTAKEPOSITION1 -> INTAKING -> INTAKEPOSITION2 -> SUBMARINE2 -> TRANSFERPOSITION -> TRANSFERING -> IDLE2 -> LIFTED -> DUMPING -> LIFTED2 -> IDLE ...
    IDLE,           // robot in parked state for travelling
    SUBMARINE,      // arm in middle position to enter submarine
    INTAKEPOSITION1, // first intake position before intaking
    INTAKING,       // spinner is intaking
    INTAKEPOSITION2, // second intake position after intaking
    SUBMARINE2,     // second submarine position after intake
    TRANSFERPOSITION, // position to position the intake to push into the bucket
    TRANSFERING, // spin intake reverse to put in bucket
    IDLE2,          // second idle state after transfering
    LIFTED,         // first lifted state before dumping
    DUMPING,        // dumping state
    LIFTED2,        // second lifted state after dumping
}


  // #endregion Enums


  // #region Drivetrain Variables
  private double DRIVETRAIN_MOTOR_SCALE = 1.0; // motor power is multiplied by this
  private double drivetrainTurboSpeedMultiplier = 1.0;
  private double drivetrainTurboSpeedTurningMultiplier = 1.0;
  private double drivetrainNormalSpeedMultiplier = 0.5;
  private double drivetrainNormalSpeedTurningMultiplier = 0.5;
  private double drivetrainSlowSpeedMultiplier = 0.25;
  private double drivetrainSlowSpeedTurningMultiplier = 0.25;
  // #endregion Drivetrain Variables


  // #region Arm Variables
  private static final double arm_kP = 0;
  private static final double arm_kI = 0;
  private static final double arm_kD = 0;
  private static final double arm_kG = 0.1;
  private static final int armTargetPositionIntaking = 500; // position for intaking
  private static final int armTargetPositionWhileEnteringOrExitingSubmarine = 400; // position for entering and exiting submarine
  private static final int armTargetPositionParking = 100; // position for parking to raise lift
  private static final int armTargetPositionWhileDepositingIntoTheDustPan = 0; // position for depositing sample into box
  private static final double armMotorPower = 0.3; // Maximum power for the arm motor
  private static final int armMinimumEncoderPosition = 0; // Joystick lower limit
  private static final int armMaximumEncoderPosition = 500; // Joystick uppper limit
  private static final double ARM_START_ANGLE = 40.0; // Starting angle in degrees
  private static final double ARM_END_ANGLE = 180.0; // End angle in degrees
  private static final double ARM_ENCODER_RANGE = 530.0; // Total encoder ticks
  private static final double DEGREES_PER_TICK = (ARM_END_ANGLE - ARM_START_ANGLE) / ARM_ENCODER_RANGE;
  //#endregion Arm Variables


  // #region Lift Variables
  private static final double lift_kP = 0.005;
  private static final double lift_kI = 0;
  private static final double lift_kD = 0;
  private static final double lift_kG = 0;
  private static final double liftMotorPower = 0.8; // Maximum power for the lift motor
  private static final int liftMinimumEncoderPosition = 0; // Joystick lower limit
  private static final int liftMaximumEncoderPosition = 6800; // Joystick upper limit
  // #endregion Lift Variables


  // #region Servo Variables
  private static final double dustpanServoPosition1 = 160.0 / 180.0; // Dustpan position 1 (converted to 0-1 range)
  private static final double dustpanServoPosition2 = 70.0 / 180.0; // Dustpan position 2 (converted to 0-1 range)
  private static final double specimenServoPosition1 = 50 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
  private static final double specimenServoPosition2 = 120.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
  // #endregion Servo Variables


  // #region Other Variables
  private static final double intakeMotorPower = 0.8; // Power for the spinner motor
  private static final int POSITION_DEADBAND = 5; // Deadband for PID control
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
      updateRobotState(); // only 1 copy for faster time between pid calls
      stateMachineActions();
      mecanumDrivetrain();
      doMovement();
      stateMachineActions();
      mecanumDrivetrain();
      doMovement();
      stateMachineActions();
      mecanumDrivetrain();
      doMovement();
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
    configureMotor(linearVerticalSlide);
    configureMotor(intakeArm);
    configureMotor(armRoller);
    dustpanServo.setPosition(dustpanServoPosition1);
    specimenIntake.setPosition(specimenServoPosition2);


    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }


  private void configureMotor(DcMotorEx motor) { // Helper function to configure motor encoder settings
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder for feedback
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Let motor be free
  }
  // #endregion Initialization Functions


  // #region TeleOP Functions


  private void teleOp() {
    handleManualOverride();
    dustpanServo();
    specimenServo();
    spinnerMotor();
    setArmPos();
    armMech();
    liftMech();
    mecanumDrivetrain();
    //telemetryUpdate();
  }


  private void handleManualOverride() { // Handles manual override on press of back button (basically polished)
    if (gamepad2.back && !lastBackButton) {
      limitsEnabled = !limitsEnabled;


    }
    lastBackButton = gamepad2.back;
  }


  private void dustpanServo() { // Handles controller inputs and setting servo power (basically polished)
    // Dustpan Servo Toggle (Right Bumper - Gamepad 2)
    if (gamepad2.right_bumper && !lastRightBumper) {
      dustpanState = !dustpanState;
      dustpanServo.setPosition(dustpanState ? dustpanServoPosition2 : dustpanServoPosition1);
    }
    lastRightBumper = gamepad2.right_bumper;
  }


  private void specimenServo() { // Handles controller inputs and setting servo power (basically polished)
    // Specimen Intake Toggle (Left Bumper - Gamepad 2)
    if (gamepad2.left_bumper && !lastLeftBumper) {
      specimenState = !specimenState;
      specimenIntake.setPosition(specimenState ? specimenServoPosition2 : specimenServoPosition1);
    }
    lastLeftBumper = gamepad2.left_bumper;
  }


  private void spinnerMotor() { // Handles controller inputs and setting motor power (basically polished)
    // Spinner Control (Gamepad 2):
    //   - A Button: Toggles the spinner forward. Pressing A again stops the spinner.
    //   - B Button: Toggles the spinner in reverse. Pressing B again stops the spinner.


    if (gamepad2.a && !lastAButton) {
      if (spinnerState == 1) {
        spinnerState = 0;
      } else {
        spinnerState = 1;
      }
    }
    lastAButton = gamepad2.a;


    if (gamepad2.b && !lastBButton) {
      if (spinnerState == -1) {
        spinnerState = 0;
      } else {
        spinnerState = -1;
      }
    }
    lastBButton = gamepad2.b;


    armRoller.setPower(spinnerState * intakeMotorPower); // apply with set power
  }


  private void setArmPos() { // Handles setting arm position from controller
    if (gamepad2.dpad_up) { // Move arm to parking position
      currentArmTarget = armTargetPositionParking;
      armPIDActive = true;
    } else if (gamepad2.dpad_right) { // Move arm to submarine entry/exit position
      currentArmTarget = armTargetPositionWhileEnteringOrExitingSubmarine;
      armPIDActive = true;
    } else if (gamepad2.dpad_down) { // Move arm to intaking position
      currentArmTarget = armTargetPositionIntaking;
      armPIDActive = true;
    } else if (gamepad2.dpad_left) { // Move arm to dustpan deposit position
      currentArmTarget = armTargetPositionWhileDepositingIntoTheDustPan;
      armPIDActive = true;
    }
  }


  private void armMech() { // Handles left joystick to move arm


    armInput = -gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y); // Square the input for finer control


    if (Math.abs(armInput) > 0.01) {
      armPIDActive = false;
      currentArmPos = intakeArm.getCurrentPosition();
      limitedArmPower = getLimitedPower(armInput * armMotorPower, currentArmPos, armMinimumEncoderPosition, armMaximumEncoderPosition);
      intakeArm.setPower(limitedArmPower);
    } else {
      if (!armPIDActive) {
        currentArmTarget = intakeArm.getCurrentPosition();
        armPIDActive = true;
        armIntegral = 0;
      }
      armPower = calculatePID(currentArmTarget, intakeArm.getCurrentPosition(), arm_kP, arm_kI, arm_kD, lastArmError, armIntegral, "arm");
      armPower = Math.max(-armMotorPower, Math.min(armMotorPower, armPower));
      armPower = getLimitedPower(armPower, intakeArm.getCurrentPosition(), armMinimumEncoderPosition, armMaximumEncoderPosition);
      intakeArm.setPower(armPower);
    }
  }


  private void liftMech() { // Handles right joystick


    liftInput = -gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y); // Square input for finer control


    if (Math.abs(liftInput) > 0.01) { // Manual movement
      slidePIDActive = false;
      currentSlidePos = linearVerticalSlide.getCurrentPosition();
      limitedSlidePower = getLimitedPower(liftInput * liftMotorPower, currentSlidePos, liftMinimumEncoderPosition, liftMaximumEncoderPosition);
      linearVerticalSlide.setPower(limitedSlidePower);
    } else { // PID Movement
      if (!slidePIDActive) {
        currentSlideTarget = linearVerticalSlide.getCurrentPosition();
        slidePIDActive = true;
        slideIntegral = 0;
      }


      slidePower = calculatePID(currentSlideTarget, linearVerticalSlide.getCurrentPosition(), lift_kP, lift_kI, lift_kD, lastSlideError, slideIntegral, "slide");
      slidePower = Math.max(-liftMotorPower, Math.min(liftMotorPower, slidePower)); // Limit power
      slidePower = getLimitedPower(slidePower, linearVerticalSlide.getCurrentPosition(), liftMinimumEncoderPosition, liftMaximumEncoderPosition); // Limit by position
      linearVerticalSlide.setPower(slidePower);
    }
  }


  private void mecanumDrivetrain() { // Handles controller inputs and setting drivetrain power (almost polished)


    // **Driving Controls (Gamepad 1):**
    // - **Left Stick (Y-axis):** Controls forward/backward movement (axial). Pushing the stick forward moves the robot forward, pulling it back moves the robot backward.
    // - **Left Stick (X-axis):** Controls sideways movement (lateral). Pushing the stick left moves the robot left, pushing it right moves the robot right.
    // - **Right Stick (X-axis):** Controls rotation (yaw). Pushing the stick left rotates the robot counter-clockwise, pushing it right rotates the robot clockwise.
    // - **Right Bumper:** Enables reduced speed (25% power).  Provides finer control for precise movements.
    // - **Right Trigger:** Enables turbo mode (100% power). Hold the trigger for maximum speed.  If not pressed or right bumper is pressed, speed is 50%.


    reducedSpeed = (gamepad1.right_bumper); // Check for right bumper pressed
    turboMode = (gamepad1.right_trigger > 0.2); // Check if the right trigger is pressed


    if (turboMode) {
      speedMultiplier = drivetrainTurboSpeedMultiplier;
    } else if (reducedSpeed) {
      speedMultiplier = drivetrainSlowSpeedMultiplier;
    } else {
      speedMultiplier = drivetrainNormalSpeedMultiplier;
    }


    // Driving controls (Gamepad 1) with scaling and speed adjustments
    axial = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) * speedMultiplier;
    lateral = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * speedMultiplier;
    yaw = -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * speedMultiplier;


    if (reducedSpeed) { // Apply cubic scaling if reduced speed is active
      axial = axial * Math.abs(gamepad1.left_stick_y) * speedMultiplier; //add the * speedmultiplier if you want it to be even slower
      lateral = lateral * Math.abs(gamepad1.left_stick_x) * speedMultiplier;
      yaw = yaw * Math.abs(gamepad1.right_stick_x) * speedMultiplier;
    }


    // Calculate individual wheel powers based on driver input. Using helper function
    powers = calculateWheelPowers(axial, lateral, yaw);


    // Set drive motor powers.
    leftFrontDrive.setPower(powers[0]);
    rightFrontDrive.setPower(powers[1]);
    leftBackDrive.setPower(powers[2]);
    rightBackDrive.setPower(powers[3]);
  }


  private void telemetryUpdate() { // Handles telemetry updates (keep off when debugging to reduce cpu loop time)
    telemetry.addData("Robot State", currentRobotState);
    telemetry.addData("Arm Target", currentArmTarget);
    telemetry.addData("Slide Target", currentSlideTarget);
    // telemetry.addData("LIMITS", limitsEnabled ? "ENABLED" : "DISABLED");
    // telemetry.addData("Arm Position", intakeArm.getCurrentPosition());
    // telemetry.addData("Arm Target", currentArmTarget);
    // telemetry.addData("Arm PID Active", armPIDActive);
    // telemetry.addData("Arm PID FF Value", ARM_PID_FF_VALUE);
    // telemetry.addData("Arm Motor Power", armPower);
    // telemetry.addData("Arm Motor PID Power", ARM_PID_OUTPUT);
    // telemetry.addData("Slide Position", linearVerticalSlide.getCurrentPosition());
    // telemetry.addData("Slide Target", currentSlideTarget);
    // telemetry.addData("Slide PID Active", slidePIDActive);
    // telemetry.addData("Dustpan State", dustpanState ? "90°" : "45°");
    // telemetry.addData("Specimen State", specimenState ? "90°" : "45°");
    // telemetry.addData("Spinner State", spinnerState == 0 ? "OFF" : (spinnerState == 1 ? "FORWARD" : "REVERSE"));
    // telemetry.addData("leftFrontDrivePower", powers[0]);
    telemetry.update();
  }


  // #endregion TeleOP Functions


  // #region State Machine
  // Handles movement between robot states
  private void updateRobotState() {
    previousState = currentState;
   
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
                currentState = RobotState.INTAKING;
                break;
            case INTAKING:
                currentState = RobotState.INTAKEPOSITION2;
                break;
            case INTAKEPOSITION2:
                currentState = RobotState.SUBMARINE2;
                break;
            case SUBMARINE2:
                currentState = RobotState.TRANSFERPOSITION;
                break;
            case TRANSFERPOSITION:
                currentState = RobotState.TRANSFERING;
                break;
            case TRANSFERING:
                currentState = RobotState.IDLE2;
                break;
            case IDLE2:
                currentState = RobotState.LIFTED;
                break;
            case LIFTED:
                currentState = RobotState.DUMPING;
                break;
            case DUMPING:
                currentState = RobotState.LIFTED2;
                break;
            case LIFTED2:
                currentState = RobotState.IDLE;
                break;
        }
    }
    else if (!gamepad2.dpad_right) {
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
            case INTAKING:
                currentState = RobotState.INTAKEPOSITION1;
                break;
            case INTAKEPOSITION2:
                currentState = RobotState.INTAKING;
                break;
            case SUBMARINE2:
                currentState = RobotState.INTAKEPOSITION2;
                break;
            case TRANSFERPOSITION:
                currentState = RobotState.SUBMARINE2;
                break;
            case TRANSFERING:
                currentState = RobotState.TRANSFERPOSITION;
                break;
            case IDLE2:
                currentState = RobotState.TRANSFERING;
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
                currentState = RobotState.LIFTED2;
                break;
        }
    }
    else if (!gamepad2.dpad_left) {
        dpad_left_pressed = false;
    }
}




  // State machine actions function
  private void stateMachineActions() {
    // Execute actions based on current state
    switch (currentRobotState) {
    case IDLE:
      // Reset all mechanisms to their default positions
      currentArmTarget = armTargetPositionParking;
      currentSlideTarget = 0;
      armPIDActive = true;
      slidePIDActive = true;
      spinnerState = 0;
      dustpanState = false;
      specimenState = false;
      break;


    case SUBMARINE:
      // Set arm to submarine position
      currentArmTarget = armTargetPositionWhileEnteringOrExitingSubmarine;
      armPIDActive = true;
      break;


//    case INTAKEPOSITION:
//      // Move arm to intake position
//      currentArmTarget = armTargetPositionIntaking;
//      armPIDActive = true;
//      specimenState = true; // Open intake
//      break;
//
//
//    case INTAKEPOSITIONAFTERINTAKING:
//      // Hold position and wait for next command
//      armPIDActive = true;
//      break;


    case INTAKING:
      // Run intake motors
      spinnerState = 1;
      armPIDActive = true;
      specimenState = true;
      break;


    case TRANSFERPOSITION:
      // Move to transfer position
      currentArmTarget = armTargetPositionWhileDepositingIntoTheDustPan;
      armPIDActive = true;
      spinnerState = 0;
      break;


    case TRANSFERING:
      // Activate dustpan
      dustpanState = true;
      specimenState = false; // Release specimen
      break;


//    case PARKED:
//      // Move to parking position
//      currentArmTarget = armTargetPositionParking;
//      armPIDActive = true;
//      break;


    case LIFTED:
      // Raise lift to maximum position
      currentSlideTarget = liftMaximumEncoderPosition;
      slidePIDActive = true;
      break;


    case DUMPING:
      // Hold position and wait for reset command
      armPIDActive = true;
      slidePIDActive = true;
      break;
    }
  }


  private void doMovement() {
    // does all movement for motors and servos


    // Apply servo positions based on states
    dustpanServo.setPosition(dustpanState ? dustpanServoPosition2 : dustpanServoPosition1);
    specimenIntake.setPosition(specimenState ? specimenServoPosition2 : specimenServoPosition1);
    armRoller.setPower(spinnerState * intakeMotorPower);


    // Run PID control for arm and slide if active
    if (armPIDActive) {
      double armPower = calculatePID(currentArmTarget, intakeArm.getCurrentPosition(),
        arm_kP, arm_kI, arm_kD, lastArmError, armIntegral, "arm");
      intakeArm.setPower(armPower);
    }


    if (slidePIDActive) {
      double slidePower = calculatePID(currentSlideTarget, linearVerticalSlide.getCurrentPosition(),
        lift_kP, lift_kI, lift_kD, lastSlideError, slideIntegral, "slide");
      linearVerticalSlide.setPower(slidePower);
    }
  }
  // #endregion State Machine


  // #region Helper Functions
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
      double feedforward = arm_kG * Math.cos(angleRadians);
      ARM_PID_OUTPUT = P + I + D;
      ARM_PID_FF_VALUE = P + I + D + feedforward;
      return P + I + D + feedforward;
    } else {
      lastSlideError = error;
      return P + I + D;
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
  // #endregion Helper Functions


}
*/
