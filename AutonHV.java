package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Anthony on 11/3/2016.
 */
//@Autonomous(name = "AutonNoBallHV v1.2", group = "Cool")
public class AutonHV extends LinearOpMode {
    Hardware10415 robot = new Hardware10415();

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = .5;
    static final double TURN_SPEED = .5;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Restting Encoders");

        resetEncoders();
        idle();
        runUsingEncoders();

        telemetry.addData("Path0", "Starting");

        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 10, 10, 5);//robot.stopper.setPosition(1);

        launcher(1, 5);
        //launcher(1, 1);
        //launcher(1, 2);

        //encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
        //encoderDrive(TURN_SPEED, -4, 4, 4.0);
        //encoderDrive(DRIVE_SPEED, 34, 34, 5.0);
        //encoderDrive(TURN_SPEED, -6, 6, 5.0);
        //encoderDrive(DRIVE_SPEED, 15, 15, 5.0);
        //encoderDrive(TURN_SPEED, -15, 15, 5.0);
        //encoderDrive(DRIVE_SPEED, -20, -20 , 5.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void setPower(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    public void resetEncoders() {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPos() {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void launcher(double speed, double timeoutS) throws InterruptedException
    {
        if(opModeIsActive()){
            runUsingEncoders();
            runtime.reset();
            //robot.motorFeeder.setPower(1);
            robot.motorLauncherLeft.setPower(speed);
            robot.motorLauncherRight.setPower(speed);
            sleep(500);
            robot.motorLift.setPower(-1);
        }
        while(opModeIsActive() && (runtime.seconds() < timeoutS)){
            telemetry.addData("Launcher", "Launching");
            telemetry.update();
            idle();
        }
        robot.motorLauncherLeft.setPower(0);
        robot.motorLauncherRight.setPower(0);
        robot.motorLift.setPower(0);
        //robot.motorFeeder.setPower(0);
        sleep(250);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            runToPos();

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            setPower(0);

            // Turn off RUN_TO_POSITION
            runUsingEncoders();

            sleep(250);   // optional pause after each move
        }
    }
}