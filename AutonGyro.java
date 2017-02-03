package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.IOException;
import java.io.PrintWriter;

/**
 * Created by Anthony on 11/30/2016.
 */
@Autonomous(name = "AutonGyro 1.1", group = "Cool")
public class AutonGyro extends LinearOpMode {
    Hardware10415 robot = new Hardware10415();
    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cColorSensor colorSensor = null;
    ModernRoboticsDigitalTouchSensor touchSensor = null;
    ModernRoboticsAnalogOpticalDistanceSensor lineSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = .5;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = .25;
    static final double TURN_SPEED = .2;

    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;

    static final double WHITE_COLOR_VALUE = .04;

    static final boolean experimentalAuton = true;

    static int gyroDistance = 0;

    //Testing variables
    static final int accelerationConstant = 3;

    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        //telemetry.setAutoClear(false);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        touchSensor = (ModernRoboticsDigitalTouchSensor) hardwareMap.touchSensor.get("touch");
        lineSensor = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("lineSensor");
        //telemetry.addData("Status", "Resetting Encoders");
        System.out.println(System.nanoTime() + " Reseting Encoders");
        colorSensor.enableLed(false);

//        gyro.calibrate();
        //while (gyro.isCalibrating()) {
        //    Thread.sleep(250);
        //    idle();
        //}

        resetEncoders();
        idle();
        runUsingEncoders();
        //while (!isStarted()) {
        //    telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
        //    telemetry.update();
        //    idle();
        //}
        //gyro.resetZAxisIntegrator();

        //telemetry.addData("Path0", "Starting");
        System.out.println(System.nanoTime() + " Path0, Starting");

        //telemetry.update();

        waitForStart();
        //robot.motorLauncherLeft.setPower(1);
        //sleep(1000);
        //robot.motorLauncherLeft.setPower(0);
        //robot.stopper.setPosition(1);
        if (experimentalAuton) {
            //launcher(1, 2.5);
            //gyroDrive(DRIVE_SPEED,10,0);
            encoderDrive(.3, -10, -10, 5);
            gyroTurn(.10, 49.5); //49
            encoderDrive(.4, -40, -40, 5);
            encoderDrive(.2, -5.5, -5.5, 5);
            encoderDrive(.1, -1.8, -1.8, 5);
            gyroTurn(.10, -54.11); //49
            whiteLine(-.15, 2);
            beacon(2.9, 2);
            //encoderDrive(.4, -30, -30, 2);
            //encoderDrive(.2, -5, -5, 2);
            //encoderDrive(.1, -1.2, -1.2, 2);
            //whiteLine(-TURN_SPEED, 5);
            //beacon(3, 2);
            gyroTurn(.10, 90);//40
            //encoderDrive(.3, 10, 10, 5);
            launcher(1, 5);
            gyroTurn(.10, 10);
            encoderDrive(.5, 45, 45, 5);



            //encoderDrive(DRIVE_SPEED, 10, 10, 5);
        } else {
            launcher(1, 2.5);
            encoderDrive(DRIVE_SPEED, 10, 10, 5);
            gyroTurn(.10, 49 + 180); //49
            encoderDrive(.3, -40, -40, 5);
            encoderDrive(.2, -5.5, -5.5, 5);
            encoderDrive(.1, -1.8, -1.8, 5);
            gyroTurn(.10, -54.12); //49
            whiteLine(-.15, 2);
            beacon(3, 2);
            encoderDrive(.3, -20, -20, 2);
            encoderDrive(.2, -2, -2, 2);
            encoderDrive(.1, -1, -1, 2);
            whiteLine(-TURN_SPEED, 5);
            beacon(3, 2);

        }


        setPower(0);

        //telemetry.addData("Path", "Complete");
        System.out.println(System.nanoTime() + " Path Complete");
        //telemetry.update();
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

    public void resetArm() throws InterruptedException
    {
        int count = 0;
        robot.motorButtonPusher.setTargetPosition(0);
        robot.motorButtonPusher.setPower(-.5);
        while(opModeIsActive() && robot.motorButtonPusher.isBusy())
        {
            /*
            count++;
            if(count % 30 == 0) {
                telemetry.addData("Touch", !touchSensor.isPressed());
                telemetry.addData("Arm Dist", robot.motorButtonPusher.getCurrentPosition());
                telemetry.update();
                idle();
            }*/
            System.out.println(System.nanoTime() + " !Touch Sensor " + !touchSensor.isPressed());
            System.out.println(System.nanoTime() + " Arm Dist " + robot.motorButtonPusher.getCurrentPosition());
            idle();
        }
        robot.motorButtonPusher.setPower(0);

    }
    public int extendArm(double dist, double power) throws InterruptedException {
        boolean pressed = false;
        int count = 0;
        //robot.motorButtonPusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(dist < 0)
        {
            robot.motorButtonPusher.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            robot.motorButtonPusher.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        int target = robot.motorButtonPusher.getCurrentPosition() + (int) (Math.abs(dist) * (COUNTS_PER_MOTOR_REV / 1.9));
        //telemetry.addData("Target", target);
        //telemetry.update();
        System.out.println(System.nanoTime() + " Arm Target " + target);
        robot.motorButtonPusher.setTargetPosition(target);
        robot.motorButtonPusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorButtonPusher.setPower(power);
        while(opModeIsActive() && robot.motorButtonPusher.isBusy()) {
            count++;
            if(count % 30 == 0) {
                System.out.println(System.nanoTime() + " !Touch " + !touchSensor.isPressed());
                System.out.println(System.nanoTime() + " Arm Dist " + robot.motorButtonPusher.getCurrentPosition());
                //telemetry.addData("!Touch", !touchSensor.isPressed());
                //telemetry.addData("Arm Dist", robot.motorButtonPusher.getCurrentPosition());
                //telemetry.update();
            }
            if(touchSensor.isPressed())
            {
                pressed = true;
                System.out.println("Pressed");
                break;
            }
            idle();
        }
        robot.motorButtonPusher.setPower(0);
        if(pressed)
        {
            robot.motorButtonPusher.setTargetPosition(robot.motorButtonPusher.getCurrentPosition() + 100);
            robot.motorButtonPusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorButtonPusher.setPower(.4);
            while(opModeIsActive() && robot.motorButtonPusher.isBusy()) {
                count++;
                if(count % 30 == 0) {
                    System.out.println(System.nanoTime() + " !Touch " + !touchSensor.isPressed());
                    System.out.println(System.nanoTime() + " Arm Dist " + robot.motorButtonPusher.getCurrentPosition());
                    System.out.println(System.nanoTime() + " In Loop True");
                            //telemetry.addData("Touch", !touchSensor.isPressed());
                    //telemetry.addData("Arm Dist", robot.motorButtonPusher.getCurrentPosition());
                    //telemetry.addData("In Loop", "True");
                    //telemetry.update();
                    idle();
                }
            }
            System.out.println(System.nanoTime() + " In Loop False");
            //telemetry.addData("In Loop", "False");

            //telemetry.update();
        }
        robot.motorButtonPusher.setPower(0);
        return (int) (robot.motorButtonPusher.getCurrentPosition() / (COUNTS_PER_MOTOR_REV / 1.9));
    }

    public void beacon(double initialArmDist, double pushDist)throws InterruptedException {
        double actualArmDist = 0;
        if(opModeIsActive()){
            encoderDrive(.15, 3.1, 3.1, 5);
            colorSensor.enableLed(false);
            extendArm(initialArmDist, .5);
            //telemetry.addData("actArmDist" , actualArmDist);
            System.out.println(System.nanoTime() + " Red " + colorSensor.red());
            //telemetry.addData("Red1", colorSensor.red());
            System.out.println(System.nanoTime() + " Blue " + colorSensor.blue());

            //telemetry.addData("Blue", colorSensor.blue());
            //telemetry.update();
            if(colorSensor.blue() < colorSensor.red() && colorSensor.red() > 4 && colorSensor.blue() < 3) {
                extendArm(pushDist, .5);
            }
            else
            {
                whiteLine(-.15, 2);
                encoderDrive(.15, -.9, -.9, 5);
                if(colorSensor.blue() < colorSensor.red() && colorSensor.red() > 4 && colorSensor.blue() < 3) {
                    extendArm(pushDist, .5);
                }
            }
            //telemetry.addData("actArmDist" , actualArmDist);
            //telemetry.addData("Red2", colorSensor.red());
            //telemetry.update();
            //sleep(2000);
            resetArm();
        }

    }

    public void launcher(double speed, double timeoutS) throws InterruptedException {
        if (opModeIsActive()) {
            runUsingEncoders();
            runtime.reset();
            //robot.motorFeeder.setPower(1);
            robot.motorLauncherLeft.setPower(speed);
            robot.motorLauncherRight.setPower(speed);
            sleep(500);
            robot.motorLift.setPower(-1);
        }
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            telemetry.addData("Launcher", "Launching");
            telemetry.update();
            idle();
        }
        robot.motorLauncherLeft.setPower(0);
        robot.motorLauncherRight.setPower(0);
        robot.motorLift.setPower(0);
        //robot.motorFeeder.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        resetEncoders();
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

                if(runtime.seconds() % 2 == 0) {// Display it for the driver.
                    //telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    //telemetry.addData("Path2", "Running at %7d :%7d",
                    //        robot.motorFrontLeft.getCurrentPosition(),
                    //        robot.motorFrontRight.getCurrentPosition());
                    //telemetry.update();
                    System.out.println(System.nanoTime() + " Path1 to " + newLeftTarget + ", " + newRightTarget);
                    System.out.println(System.nanoTime() + " Path2 at " + robot.motorFrontLeft.getCurrentPosition() + ", " + robot.motorFrontRight.getCurrentPosition());

                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            setPower(0);

            // Turn off RUN_TO_POSITION
            runUsingEncoders();
            // optional pause after each move
        }
    }
    public void whiteLine(double speed, double timeoutS) throws InterruptedException {
        // Ensure that the opmode is still active
        if (opModeIsActive()){
            runUsingEncoders();
            lineSensor.enableLed(true);
            // reset the timeout time and start motion.
            runtime.reset();
            setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && lineSensor.getLightDetected() < WHITE_COLOR_VALUE) {

                // Display it for the driver.
                //telemetry.addData("Path1", lineSensor.getRawLightDetected());
                //telemetry.addData("Path2", "Running at %7d :%7d",
                //        robot.motorFrontLeft.getCurrentPosition(),
                //        robot.motorFrontRight.getCurrentPosition());
                //telemetry.update();
                System.out.println(System.nanoTime() + " WhiteLine " + lineSensor.getLightDetected());
                System.out.println(System.nanoTime() + " Path2 at " + robot.motorFrontLeft.getCurrentPosition() + ", " + robot.motorFrontRight.getCurrentPosition());

                        // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            setPower(0);

            // optional pause after each move
        }
    }

    public void turnToZero(double turnSpeed) throws InterruptedException
    {
        runUsingEncoders();
        robot.motorBackRight.setPower(-turnSpeed);
        robot.motorBackLeft.setPower(turnSpeed);
        robot.motorFrontLeft.setPower(turnSpeed);
        robot.motorFrontRight.setPower(-turnSpeed);

        while(gyro.getIntegratedZValue() <= 0 - HEADING_THRESHOLD && gyro.getIntegratedZValue() <= 0 + HEADING_THRESHOLD)
        {
            idle();
        }

        setPower(0);
        runUsingEncoders();
    }

    public void acceleratedDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS, int startingSpeed, int maxSpeed) throws InterruptedException
    {
        resetEncoders();
        int newLeftTarget;
        int newRightTarget;
        double accelDist = 10;
        double deccelDist = 10;
        //Left inch and right inch have to be the same for this method to work!!!!!!
        double constDist = leftInches-accelDist-deccelDist;
        int currentSpeed = startingSpeed;
        double oldTime = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (accelDist * COUNTS_PER_INCH);
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (accelDist * COUNTS_PER_INCH);
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            runToPos();
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {
                if(currentSpeed <= maxSpeed) {
                    if(robot.motorFrontLeft.getCurrentPosition() % 1120 == 0) //Needs + - value
                    currentSpeed += accelerationConstant;
                    setPower(currentSpeed);
                    oldTime = runtime.milliseconds();
                }
                // Allow time for other processes to run.
                idle();
            }

            //CONSTANT SPEED
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (constDist * COUNTS_PER_INCH);
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (constDist * COUNTS_PER_INCH);
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);
            runToPos();
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {
                idle();
            }

            //DECCELERATION
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (deccelDist * COUNTS_PER_INCH);
            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (deccelDist * COUNTS_PER_INCH);
            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);
            runToPos();
            runtime.reset();
            oldTime = 0.0;
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {
                if(currentSpeed >= 0  && runtime.milliseconds() - .250 > oldTime) {
                    currentSpeed -= accelerationConstant;
                    setPower(currentSpeed);
                    oldTime = runtime.milliseconds();

                }
                idle();
            }





            // keep looping while we are still active, and there is time left, and both motors are running.

            setPower(0);
            runUsingEncoders();
            }
    }

    public void gyroDrive(double speed,
                          double distance,
                          double angle) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
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
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorFrontLeft.setPower(leftSpeed);
                robot.motorBackLeft.setPower(leftSpeed);
                robot.motorBackRight.setPower(rightSpeed);
                robot.motorFrontRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
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
    public void gyroTurn2(double speed, double targetHeading, double timeout)
    {
        runtime.reset();
        int currentHeading = gyro.getIntegratedZValue();
        if(targetHeading > 0) {
            while (currentHeading < targetHeading && runtime.seconds() < timeout) {
                currentHeading = gyro.getIntegratedZValue();
                telemetry.addData("Current Heading", currentHeading);
                telemetry.addData("Target", targetHeading);
                //if(currentHeading > 180){
                //    currentHeading = currentHeading - 360;
                //}
                //double headingError = targetHeading - currentHeading;
                //double driveSteering = getSteer(headingError, .7);

                double leftPower = speed;
                double rightPower = -leftPower;

                robot.motorFrontLeft.setPower(leftPower);
                robot.motorBackLeft.setPower(leftPower);
                robot.motorFrontRight.setPower(rightPower);
                robot.motorBackRight.setPower(rightPower);
            }
        }
        else
        {
            while(currentHeading > targetHeading && runtime.seconds() < timeout)
            {
                currentHeading = gyro.getIntegratedZValue();
                telemetry.addData("Current Heading", currentHeading);
                telemetry.addData("Target", targetHeading);
                //double headingError = targetHeading + currentHeading;
                //double driveSteering = getSteer(headingError, .7);

                double rightPower = speed;
                double leftPower = -rightPower;

                robot.motorFrontLeft.setPower(leftPower);
                robot.motorBackLeft.setPower(leftPower);
                robot.motorFrontRight.setPower(rightPower);
                robot.motorBackRight.setPower(rightPower);
            }
        }
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
    }

    public int gyroDelta()
    {
        return gyroDistance - gyro.getIntegratedZValue();
    }

    public void resetDelta()
    {
        gyroDistance = gyro.getIntegratedZValue();
    }

    public void gyroTurn(double power, double degrees) throws InterruptedException
    {
        resetDelta();
        float direction = Math.signum((float)(degrees));

        if(direction == 0)
        {
            return;
        }

        robot.motorFrontLeft.setPower(-1*direction*power);
        robot.motorFrontRight.setPower(direction*power);
        robot.motorBackRight.setPower(direction*power);
        robot.motorBackLeft.setPower(-1*direction*power);

        while(Math.abs(gyroDelta()) < Math.abs(degrees))
        {
            idle();
        }

        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorBackLeft.setPower(0);
    }

    public void gyroTurn3(double speed, double angle)
            throws InterruptedException {

            gyroTurnPos(speed,angle);
    }

    public void gyroTurnPos(double speed, double angle) throws InterruptedException {

        while (opModeIsActive() && !onHeading(speed,angle,P_TURN_COEFF))
        {
            idle();
            telemetry.addLine("Angle " + gyro.getHeading());

        }
    }

    public void gyroTurnNeg(double speed, double angle) throws InterruptedException
    {

        double newAngle = Math.abs(angle);

        //robot.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive() && ((newAngle - 3 < gyro.getHeading() && (gyro.getHeading() < newAngle + 3)))) {
            robot.motorFrontRight.setPower(speed);
            robot.motorFrontLeft.setPower(speed);
            robot.motorBackLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);
            idle();
            telemetry.addLine("Angle " + gyro.getHeading());

        }

        //telemetry.addLine("Angle " + gyro.getHeading());
        robot.motorFrontRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);

        robot.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
        robot.motorFrontRight.setPower(rightSpeed);
        robot.motorBackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void parallelPark() throws InterruptedException
    {
        //gyroTurn(TURN_SPEED,45);

        double leftPower = 25;
        double rightPower = 50;
        double angle = 0;

        while((angle - 5 < gyro.getHeading() && gyro.getHeading() < angle + 5))
        {
            robot.motorFrontLeft.setPower(leftPower);
            robot.motorBackLeft.setPower(leftPower);
            robot.motorFrontRight.setPower(rightPower);
            robot.motorBackRight.setPower(rightPower);
        }

        robot.motorFrontRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }

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

    public void pushButton(double dist,int timeOut) throws InterruptedException
    {
        robot.motorButtonPusher.setPower(.2);

        while(opModeIsActive() && (runtime.seconds() < timeOut) && !touchSensor.isPressed())
        {
            idle();
        }
        robot.motorButtonPusher.setPower(0);
    }

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
        robot.motorFrontRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }


}

