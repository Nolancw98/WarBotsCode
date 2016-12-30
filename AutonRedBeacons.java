package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Anthony on 11/30/2016.
 */
@Autonomous(name = "AutonRedBeancon", group = "Cool")
public class AutonRedBeacons extends LinearOpMode{
    Hardware10415 robot = new Hardware10415();
    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cColorSensor colorSensor = null;
    ModernRoboticsAnalogOpticalDistanceSensor lineSensor = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = .3;
    static final double TURN_SPEED = .3;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final int WHITE_COLOR_VALUE = 120;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        lineSensor = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("lineSensor");
        telemetry.addData("Status", "Restting Encoders");
        gyro.calibrate();
        while (gyro.isCalibrating()){
            Thread.sleep(100);
            idle();
        }

        resetEncoders();
        idle();
        runUsingEncoders();
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        telemetry.addData("Path0", "Starting");

        telemetry.update();

        waitForStart();
        robot.button.setPosition(.45);

        //robot.stopper.setPosition(1);
        //launcher(1, 5);
        gyroDrive(DRIVE_SPEED,5,0);
        gyroTurn(TURN_SPEED,-60); //turn left 60
        gyroHold(TURN_SPEED,-60,.5);
        gyroDrive(DRIVE_SPEED, 60,0); //drive to the tile with the white line

        gyroTurn(TURN_SPEED,5);//turn to face the white line (should be about perpendicular to it
        gyroHold(TURN_SPEED,5,.5);
        if(findWhiteLine(DRIVE_SPEED/2,24,0,5))
        {
            IfRedBeacon(WHITE_COLOR_VALUE);
        }

        //gyroDrive(DRIVE_SPEED,36,0);
        //if(findWhiteLine(DRIVE_SPEED/2,12,0,5))
        //{
        //    IfRedBeacon(120);
        //}*/

        //END

        //gyroDrive(DRIVE_SPEED/2.0,12,0); // drive to the while line at slow speed


        /*
        gyroDrive(DRIVE_SPEED, 10, 0);
        gyroTurn(TURN_SPEED, -10);
        gyroHold(TURN_SPEED, -10.0, 0.5);
        gyroDrive(DRIVE_SPEED, 34, -10);
        gyroTurn(TURN_SPEED, -25);
        gyroHold(TURN_SPEED, -25, .5);
        gyroDrive(DRIVE_SPEED, 15, -25);
        gyroTurn(TURN_SPEED, -45);
        gyroHold(TURN_SPEED, -45, .5);
        gyroDrive(DRIVE_SPEED, 20, 180);*/
        setPower(0);

        //launcher(1, 1);
        //launcher(1, 2);
            /*
            encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
            encoderDrive(TURN_SPEED, -4, 4, 4.0);
            encoderDrive(DRIVE_SPEED, 34, 34, 5.0);
            encoderDrive(TURN_SPEED, -6, 6, 5.0);
            encoderDrive(DRIVE_SPEED, 15, 15, 5.0);
            encoderDrive(TURN_SPEED, -15, 15, 5.0);
            encoderDrive(DRIVE_SPEED, -20, -20 , 5.0);
            */
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public boolean findWhiteLine(double speed, double distance, double angle, int timeOut) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        lineSensor.enableLed(true);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);

            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (timer.seconds() < timeOut && opModeIsActive() &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorFrontLeft.setPower(leftSpeed);
                robot.motorBackLeft.setPower(leftSpeed);
                robot.motorBackRight.setPower(rightSpeed);
                robot.motorFrontRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();

                // Allow time for other processes to run.
                idle();

                //check for line
                if(lineSensor.getRawLightDetected() > WHITE_COLOR_VALUE)
                {
                    runUsingEncoders();
                    setPower(0);
                    return true;
                }
            }

            // Stop all motion;
            setPower(0);

            // Turn off RUN_TO_POSITION
            runUsingEncoders();
        }
        return false;
    }

    public void IfRedBeacon(int redLevel) throws InterruptedException
    {
        if(colorSensor.red() > redLevel)
        {
            robot.button.setPosition(.7);
            wait(1000);
        }
        else
        {
            robot.button.setPosition(.3);
            wait(1000);
        }

        robot.button.setPosition(.5);
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
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);

            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorFrontLeft.setPower(leftSpeed);
                robot.motorBackLeft.setPower(leftSpeed);
                robot.motorBackRight.setPower(rightSpeed);
                robot.motorFrontRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            setPower(0);

            // Turn off RUN_TO_POSITION
            runUsingEncoders();
        }
    }
    public void gyroTurn (  double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold( double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorFrontLeft.setPower(leftSpeed);
        robot.motorBackLeft.setPower(leftSpeed);
        robot.motorBackRight.setPower(rightSpeed);
        robot.motorFrontRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}