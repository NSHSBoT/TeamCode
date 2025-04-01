/*package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Test: Arm Position OpMode", group = "Linear OpMode")
public class TestOmniOpMode_Linear extends LinearOpMode {
    // Easily adjustable constants at the top
    private static final int ARM_TARGET_POSITION_INTAKING = 500;                    // X button - position for intaking
    private static final int ARM_TARGET_POSITION_SUBMARINE = 400;                   // Y button - position for submarine
    private static final int ARM_TARGET_POSITION_PARKING = 100;                     // A button - position for parking
    private static final int ARM_TARGET_POSITION_DUSTPAN = 0;                       // B button - position for dustpan


    // PID Constants - adjust these for tuning
    private static final double ARM_KP = 0.01;                                      // Proportional gain
    private static final double ARM_KI = 0.0;                                       // Integral gain
    private static final double ARM_KD = 0.0;                                       // Derivative gain
    private static final double ARM_KG = 0.1;                                       // Gravity compensation
    private static final int POSITION_DEADBAND = 20;                                 // Acceptable error range
    private static final double MAX_POWER = 0.2;                                    // Maximum motor power


    // Motor and timing variables
    private DcMotorEx armMotor;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0;
    private double integral = 0;
    private int currentTarget = 0;


    @Override
    public void runOpMode() {
        // Initialize the arm motor
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        pidTimer.reset();


        while (opModeIsActive()) {
            // Update target position based on gamepad buttons
            if (gamepad1.x) {
                currentTarget = ARM_TARGET_POSITION_INTAKING;
            } else if (gamepad1.y) {
                currentTarget = ARM_TARGET_POSITION_SUBMARINE;
            } else if (gamepad1.a) {
                currentTarget = ARM_TARGET_POSITION_PARKING;
            } else if (gamepad1.b) {
                currentTarget = ARM_TARGET_POSITION_DUSTPAN;
            }


            // Calculate and apply PID
            double power = calculateArmPID(currentTarget, armMotor.getCurrentPosition());
            armMotor.setPower(power);


            // Update telemetry
            telemetry.addData("Target Position", currentTarget);
            telemetry.addData("Current Position", armMotor.getCurrentPosition());
            telemetry.addData("Motor Power", power);
            telemetry.addData("Error", currentTarget - armMotor.getCurrentPosition());
            telemetry.update();
        }
    }


    private double calculateArmPID(int targetPosition, int currentPosition) {
        double error = targetPosition - currentPosition;


        // Return 0 power if within deadband
        if (Math.abs(error) <= POSITION_DEADBAND) {
            return 0.0;
        }


        // Calculate PID components
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();


        // Proportional term
        double P = error * ARM_KP;


        // Integral term
        integral += error * deltaTime;
        integral = Math.max(-MAX_POWER, Math.min(MAX_POWER, integral));
        double I = integral * ARM_KI;


        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double D = derivative * ARM_KD;
        lastError = error;


        // Gravity compensation
        double currentAngle = 40.0 + (currentPosition * (180.0 - 40.0) / 530.0);
        double angleRadians = Math.toRadians(currentAngle);
        double G = ARM_KG * Math.cos(angleRadians);


        // Combine terms and limit power
        double power = P + I + D + G;
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
    }
}
*/


