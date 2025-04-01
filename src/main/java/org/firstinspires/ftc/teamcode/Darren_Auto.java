package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Autonomous(name = "Auto 2 Piece", group = "Robot")
//@Disabled
public class Darren_Auto extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime(); // Timer for tracking runtime
    private DcMotor leftFrontDrive2 = null; // Motor for left front wheel
    private DcMotor leftBackDrive2 = null; // Motor for left back wheel
    private DcMotor rightFrontDrive2 = null; // Motor for right front wheel
    private DcMotor rightBackDrive2 = null; // Motor for right back wheel
    private DcMotorEx linearVerticalSlide2 = null; // Motor for linear slide (with extended functionality)
    private DcMotorEx intakeArm2 = null; // Motor for intake arm (with extended functionality)
    private DcMotorEx armRoller2 = null; // Motor for arm roller (with extended functionality)
    public Servo specimenIntake; // Servo for specimen intake
    public Servo dustpanServo; // Servo for dustpan
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
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastStartButton = false;
    private boolean firstArmMotorCalibrate = true;
    private double DRIVETRAIN_MOTOR_SCALE = 1.0; // motor power is multiplied by this
    private double drivetrainTurboSpeedMultiplier = 1.0;
    private double drivetrainTurboSpeedTurningMultiplier = 1.0;
    private double drivetrainNormalSpeedMultiplier = 0.5;
    private double drivetrainNormalSpeedTurningMultiplier = 0.5;
    private double drivetrainSlowSpeedMultiplier = 0.5;
    private double drivetrainSlowSpeedTurningMultiplier = 0.25;
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
    private double previousError = 0.0; // Class member to store previous error
    private double errorSum = 0.0; // Class member to store integrated error
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
    private static final int FF_ZONE1_END = 160; // End of zero power zone
    private static final int FF_ZONE2_END = 450; // End of increasing power zone
    private static final int FF_ZONE3_END = 560; // End of decreasing power zone
    private static final double FF_MAX_POWER = 0.2; // Max feedforward power
    private static final double FF_ZONE1_MAX_POWER = 0.1; // Adjust starting zone power
    private static final double lift_kP = 0.002;
    private static final double lift_kI = 0;
    private static final double lift_kD = 0.00;
    private static final double lift_kG = 0;
    private static final double liftMotorPower = 1; // Maximum power for the lift motor
    private static final double liftMotorPowerDown = 0.4;
    private static final int liftMinimumEncoderPosition = 0; // Joystick lower limit
    private static final int liftMaximumEncoderPosition = 6000; // Joystick upper limit
    private static final int liftTopStageEncoderPosition = 4300;
    private static final double dustpanServoPositionIntaking = 145 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double dustpanServoPositionDumping = 50.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
    private static final double specimenServoPositionClosed = 120 / 180.0; // Specimen intake position 1 (converted to 0-1 range)
    private static final double specimenServoPositionOpen = 50.0 / 180.0; // Specimen intake position 2 (converted to 0-1 range)
    private static final double intakeMotorPower = 1; // Power for the spinner motor
    private static final int POSITION_DEADBAND = 0; // do not change from 0 or the robot will act like it drank 10 cups of coffee
    private static final double INTEGRAL_LIMIT = armMotorPower; // Match to max power
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
        IDLE(50),
        SPECIMENTRANSFER(2000), // Example position - adjust as needed
        SAMPLETRANSFER(4500);


        public final int targetPosition;


        LiftState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }


    static double FORWARD_SPEED = -0.4;
    static final double STRAFE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;
    double secondsPerInch = 0.05;
    double secondsPerInchStrafe = 0.05;
    double secondsPer90Degrees = 0.7;
    double voltageshow;
    double timeRatio;
    double secondsPerDegree = secondsPer90Degrees / 90;
    boolean moveComplete = false;



    boolean step1Complete = false;
    boolean step2Complete = false;
    boolean step3Complete = false;
    boolean step4Complete = false;
    boolean step5Complete = false;
    boolean step6Complete = false;
    boolean step7Complete = false;
    boolean step8Complete = false;
    boolean step9Complete = false;
    boolean step10Complete = false;
    boolean step11Complete = false;
    boolean step12Complete = false;
    boolean step13Complete = false;
    boolean step14Complete = false;
    boolean step15Complete = false;
    boolean step16Complete = false;
    boolean step17Complete = false;
    boolean step18Complete = false;
    boolean step19Complete = false;
    boolean step20Complete = false;
    boolean step21Complete = false;
    boolean step22Complete = false;
    boolean step23Complete = false;
    boolean step24Complete = false;
    boolean step25Complete = false;


    private ElapsedTime autonomousTimer = new ElapsedTime();
    private LiftState currentLiftState = LiftState.GROUND;
    private ArmState currentArmState = ArmState.IDLE;


    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontDrive2 = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive2 = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive2 = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive2 = hardwareMap.get(DcMotor.class, "back_right");
        linearVerticalSlide2 = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        intakeArm2 = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armRoller2 = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        dustpanServo = hardwareMap.get(Servo.class, "bucket");
        specimenIntake = hardwareMap.get(Servo.class, "claw");


        // Set motor directions
        leftFrontDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive2.setDirection(DcMotor.Direction.FORWARD);


        // Set zero power behavior to BRAKE
        leftFrontDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Configure motors
        configureMotor(linearVerticalSlide2);
        //linearVerticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        configureMotor(intakeArm2);
        configureMotor(armRoller2);

        //private  VoltageSensor myControlHubVoltageSensor;
        double voltageshow;
        double timeRatio = 1;
        //voltageshow = myControlHubVoltageSensor.getVoltage();
//        if (voltageshow >= 13) {timeRatio=1;}
//        if (voltageshow < 13 && voltageshow >=12.5) {timeRatio=1.25;}
//        if (voltageshow < 12.5 && voltageshow >= 12) {timeRatio=1.5;}
        secondsPerInch = secondsPerInch*timeRatio;
        secondsPerInchStrafe = secondsPerInchStrafe*timeRatio;
        secondsPer90Degrees = secondsPer90Degrees*timeRatio;


        // Wait for start
        waitForStart();
        autonomousTimer.reset();


        // Main loop
        while (opModeIsActive()) {
            auton();
            doMovement();
        }
    }


    private void auton() {
        double time = autonomousTimer.seconds();


        double step1Duration = 0.2; // park arm and lift
        double step2Duration = 2.0; // strafe left
        double step3Duration = 2.0; // move arm and lift to high basket
        double step4Duration = 2.3; // move back and extend bucket
        double step5Duration = 0.1; // ignore do nothing
        double step6Duration = 0.5; // retract bucket and go back
        double step7Duration = 0.5; // ignore do nothing
        double step8Duration = 2.0; // move lift to ground
        double step9Duration = 1.0; // rotate to first sample
        double step10Duration = 2.5; // move intake to intake position
        double step11Duration = 2.5;  // intake block and move forward
        double step12Duration = 2.5; // go backward and stop intaking
        double step13Duration = 1.0; // move arm to transfer state
        double step14Duration = 2.0; // outtake sample
        double step15Duration = 0.1; // stop outtaking
        double step16Duration = 2.0; // turn back
        double step17Duration = 3.0; // move arm to idle position and lift up to sample position
        double step18Duration = 2.0; // extend dustpan and drive back
        double step19Duration = 0.5; // retract dustpan and drive forward
        double step20Duration = 1.0; // move lift to ground and arm to idle
        double step21Duration = 1.0; // park arm
        double step22Duration = 1.0; //
        double step23Duration = 1.0; //
        double step24Duration = 1.0; //
        double step25Duration = 1.0; //

        double step1Start = 0.0;
        double step1End = step1Start + step1Duration;


        double step2Start = step1End;
        double step2End = step2Start + step2Duration;


        double step3Start = step2End;
        double step3End = step3Start + step3Duration;


        double step4Start = step3End;
        double step4End = step4Start + step4Duration;


        double step5Start = step4End;
        double step5End = step5Start + step5Duration;


        double step6Start = step5End;
        double step6End = step6Start + step6Duration;


        double step7Start = step6End;
        double step7End = step7Start + step7Duration;


        double step8Start = step7End;
        double step8End = step8Start + step8Duration;


        double step9Start = step8End;
        double step9End = step9Start + step9Duration;


        double step10Start = step9End;
        double step10End = step10Start + step10Duration;


        double step11Start = step10End;
        double step11End = step11Start + step11Duration;


        double step12Start = step11End;
        double step12End = step12Start + step12Duration;


        double step13Start = step12End;
        double step13End = step13Start + step13Duration;


        double step14Start = step13End;
        double step14End = step14Start + step14Duration;


        double step15Start = step14End;
        double step15End = step15Start + step15Duration;


        double step16Start = step15End;
        double step16End = step16Start + step16Duration;


        double step17Start = step16End;
        double step17End = step17Start + step17Duration;


        double step18Start = step17End;
        double step18End = step18Start + step18Duration;


        double step19Start = step18End;
        double step19End = step19Start + step19Duration;


        double step20Start = step19End;
        double step20End = step20Start + step20Duration;


        double step21Start = step20End;
        double step21End = step21Start + step21Duration;


        double step22Start = step21End;
        double step22End = step22Start + step22Duration;


        double step23Start = step22End;
        double step23End = step23Start + step23Duration;


        double step24Start = step23End;
        double step24End = step24Start + step24Duration;


        double step25Start = step24End;
        double step25End = step25Start + step25Duration;


        if (time > step1Start && time < step1End && !step1Complete) {
            currentArmState = ArmState.TRANSFER;
            currentLiftState = LiftState.GROUND;
            dustpanState = false;
            step1Complete = true;
        }
        if (time > step2Start && time < step2End && !step2Complete) {
            strafeLeft(13);
            step2Complete = true;
        }
        if (time > step3Start && time < step3End && !step3Complete) {
            // step 3 move arm and lift to high basket
            currentArmState = ArmState.IDLE;
            currentLiftState = LiftState.SAMPLETRANSFER;
            dustpanState = false;
            step3Complete = true;
        }
        if (time > step4Start && time < step4End && !step4Complete) {
            // step 4 move back
            dustpanState = true;
            drive(-5);
            step4Complete = true;
        }
        if (time > step5Start && time < step5End && !step5Complete) {
            step5Complete = true;
        }
        if (time > step6Start && time < step6End && !step6Complete) {
            dustpanState = false;
            drive(3);
            step6Complete = true;
        }
        if (time > step7Start && time < step7End && !step7Complete) {

            step7Complete = true;
        }
        if (time > step8Start && time < step8End && !step8Complete) {
            currentLiftState = LiftState.GROUND;
            step8Complete = true;
        }
        if (time > step9Start && time < step9End && !step9Complete) {
            turn(60);
            step9Complete = true;
        }
        if (time > step10Start && time < step10End && !step10Complete) {
            currentArmState = ArmState.INTAKE;
            FORWARD_SPEED = -0.2;
            drive(-4);
            step10Complete = true;
        }
        if (time > step11Start && time < step11End && !step11Complete) {
            armRoller2.setPower(1 * intakeMotorPower);
            FORWARD_SPEED = -0.6;
            drive(10);

            step11Complete = true;
        }
        if (time > step12Start && time < step12End && !step12Complete) {
            // Add your code for step 12 here
            armRoller2.setPower(0);
            FORWARD_SPEED = -0.2;
            drive(-30);


            step12Complete = true;
        }
        if (time > step13Start && time < step13End && !step13Complete) {
            currentArmState = ArmState.TRANSFER;
            step13Complete = true;
        }
        if (time > step14Start && time < step14End && !step14Complete) {
            armRoller2.setPower(-0.3 * intakeMotorPower);

            step14Complete = true;
        }
        if (time > step15Start && time < step15End && !step15Complete) {
            armRoller2.setPower(0);
            step15Complete = true;
        }
        if (time > step16Start && time < step16End && !step16Complete) {
            // turning after depositing
            turn(-60);

            step16Complete = true;
        }
        if (time > step17Start && time < step17End && !step17Complete) {
            //
            currentArmState = ArmState.IDLE;
            currentLiftState = LiftState.SAMPLETRANSFER;
            drive(2);
            step17Complete = true;
        }
        if (time > step18Start && time < step18End && !step18Complete) {
            // Add your code for step 18 here
            dustpanState = true;
            drive(-3);


            step18Complete = true;
        }
        if (time > step19Start && time < step19End && !step19Complete) {
            // Add your code for step 19 here
            drive(7);
            dustpanState = false;

            step19Complete = true;
        }
        if (time > step20Start && time < step20End && !step20Complete) {
            // Add your code for step 20 here
            currentLiftState = LiftState.GROUND;
            currentArmState = ArmState.IDLE;

            step20Complete = true;
        }
        if (time > step21Start && time < step21End && !step21Complete) {
            // Add your code for step 21 here
            currentArmState = ArmState.IDLE;
            step21Complete = true;
        }
        if (time > step22Start && time < step22End && !step22Complete) {
            // Add your code for step 22 here
            step22Complete = true;
        }
        if (time > step23Start && time < step23End && !step23Complete) {
            // Add your code for step 23 here
            step23Complete = true;
        }
        if (time > step24Start && time < step24End && !step24Complete) {
            // Add your code for step 24 here
            step24Complete = true;
        }
        if (time > step25Start && time < step25End && !step25Complete) {
            // Add your code for step 25 here
            step25Complete = true;
        }
    }
    private void strafeLeft(double inches) {
        strafe(inches); // Negative inches to indicate left movement
    }


    private void strafeRight(double inches) {
        strafe(-inches); // Positive inches to indicate right movement
    }


    private void strafe(double inches) {
        double time = Math.abs(inches) * secondsPerInchStrafe;
        double direction = Math.signum(inches);
        runtime.reset();


        while (runtime.seconds() < time && opModeIsActive()) {
            leftFrontDrive2.setPower(direction * STRAFE_SPEED);
            leftBackDrive2.setPower(-direction * STRAFE_SPEED);
            rightFrontDrive2.setPower(-direction * STRAFE_SPEED);
            rightBackDrive2.setPower(direction * STRAFE_SPEED);
        }


        leftFrontDrive2.setPower(0);
        leftBackDrive2.setPower(0);
        rightFrontDrive2.setPower(0);
        rightBackDrive2.setPower(0);
        stopDriving();

        moveComplete = true;
    }


    public void drive(double inches) {
        double time = Math.abs(inches) * secondsPerInch; // Use absolute value for time
        double direction = Math.signum(inches); // +1 for forward, -1 for backward
        runtime.reset();


        while (runtime.seconds() < time && opModeIsActive()) {
            leftFrontDrive2.setPower(direction * FORWARD_SPEED);
            leftBackDrive2.setPower(direction * FORWARD_SPEED);
            rightFrontDrive2.setPower(direction * FORWARD_SPEED);
            rightBackDrive2.setPower(direction * FORWARD_SPEED);
        }


        // Stop the motors after the movement
        leftFrontDrive2.setPower(0);
        leftBackDrive2.setPower(0);
        rightFrontDrive2.setPower(0);
        rightBackDrive2.setPower(0);


        moveComplete = true;
    }


    private void turn(double degrees) {
        runtime.reset();


        // Determine the direction of the turn
        double direction = Math.signum(degrees);


        // Calculate the target time based on the desired angle
        double secondsForTurn = Math.abs(degrees) * secondsPerDegree;


        while (runtime.seconds() < secondsForTurn && opModeIsActive()) {
            leftFrontDrive2.setPower(direction * TURN_SPEED);
            leftBackDrive2.setPower(direction * TURN_SPEED);
            rightFrontDrive2.setPower(-direction * TURN_SPEED);
            rightBackDrive2.setPower(-direction * TURN_SPEED);
        }


        // Stop the motors after the turn
        leftFrontDrive2.setPower(0);
        leftBackDrive2.setPower(0);
        rightFrontDrive2.setPower(0);
        rightBackDrive2.setPower(0);


        moveComplete = true;
    }


    private void stopDriving() {
        leftFrontDrive2.setPower(0);
        leftBackDrive2.setPower(0);
        rightFrontDrive2.setPower(0);
        rightBackDrive2.setPower(0);
    }


    private void doMovement() {


        liftCurrentEncoderPosition = linearVerticalSlide2.getCurrentPosition();
        double slidePower = calculateSlideP();
        linearVerticalSlide2.setPower(slidePower);
        armCurrentEncoderPosition = intakeArm2.getCurrentPosition();
        double armPower = calculateArmPIDG();
        intakeArm2.setPower(armPower);
        if (dustpanState) {
            dustpanServo.setPosition(dustpanServoPositionDumping); // Position 2 for dumping
        } else {
            dustpanServo.setPosition(dustpanServoPositionIntaking); // Position 1 for intaking
        }


    }


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
        double feedforward = arm_kG * Math.cos(angleRadians); // Gravity compensation
        double linearFF = 0;
        if (armCurrentEncoderPosition <= FF_ZONE1_END) {
            double zoneProgress = armCurrentEncoderPosition / (double) FF_ZONE1_END;
            linearFF = zoneProgress * FF_ZONE1_MAX_POWER;
        } else if (armCurrentEncoderPosition <= FF_ZONE2_END) {
            double zoneProgress = (armCurrentEncoderPosition - FF_ZONE1_END) / (double)(FF_ZONE2_END - FF_ZONE1_END);
            linearFF = FF_ZONE1_MAX_POWER + (zoneProgress * (FF_MAX_POWER - FF_ZONE1_MAX_POWER));
        } else if (armCurrentEncoderPosition <= FF_ZONE3_END) {
            double zoneProgress = (armCurrentEncoderPosition - FF_ZONE2_END) / (double)(FF_ZONE3_END - FF_ZONE2_END);
            linearFF = FF_MAX_POWER * (1 - zoneProgress);
        }
        double output = P + I + D + feedforward + linearFF;
//        if (currentArmState == ArmState.TRANSFER && armCurrentEncoderPosition < 400 && armCurrentEncoderPosition >= 155) {
//            armMotorPower = armMotorPowerSuperLow;
//        } else if (armCurrentEncoderPosition < 155 && (currentArmState == ArmState.TRANSFER)) {
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

    private void configureMotor(DcMotorEx motor) { // Helper function to configure motor encoder settings
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Do not use encoder for feedback


    }

}





