/*package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Test: P Only Diagnostic", group = "Linear OpMode")
public class TestOpMode_Linear extends LinearOpMode {
    // Simplified constants for testing
    private static final int ARM_TARGET_POSITION_INTAKING = 500;
    private static final int ARM_TARGET_POSITION_SUBMARINE = 400;
    private static final int ARM_TARGET_POSITION_PARKING = 100;
    private static final int ARM_TARGET_POSITION_DUSTPAN = 0;


    private static final double ARM_KP = 0.01;
    private static final int POSITION_DEADBAND = 5;
    private static final double MAX_POWER = 0.5;


    private DcMotorEx armMotor;
    private ElapsedTime diagnosticTimer = new ElapsedTime();
    private int lastPosition = 0;
    private int currentTarget = 0;


    @Override
    public void runOpMode() {
        // Initialize the arm motor
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Start with motor power at 0
        armMotor.setPower(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        diagnosticTimer.reset();


        while (opModeIsActive()) {
            int currentPosition = armMotor.getCurrentPosition();

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


            // Calculate error and P-only power
            double error = currentTarget - currentPosition;
            double power = 0;

            if (Math.abs(error) > POSITION_DEADBAND) {
                power = error * ARM_KP;
                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            }


            // Apply power
            armMotor.setPower(power);


            // Calculate velocity (positions per second)
            double deltaTime = diagnosticTimer.seconds();
            double velocity = 0;
            if (deltaTime > 0) {
                velocity = (currentPosition - lastPosition) / deltaTime;
            }
            diagnosticTimer.reset();
            lastPosition = currentPosition;


            // Detailed telemetry for debugging
            telemetry.addLine("=== Position Info ===");
            telemetry.addData("Target Position", currentTarget);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);

            telemetry.addLine("\n=== Motor Info ===");
            telemetry.addData("Calculated Power", power);
            telemetry.addData("Actual Motor Power", armMotor.getPower());
            telemetry.addData("Motor Mode", armMotor.getMode());
            telemetry.addData("Direction", error > 0 ? "Moving Up" : "Moving Down");

            telemetry.addLine("\n=== Motion Info ===");
            telemetry.addData("Velocity (pos/sec)", "%.2f", velocity);
            telemetry.addData("Delta Time (sec)", "%.4f", deltaTime);


            telemetry.update();
        }
    }
}

*/