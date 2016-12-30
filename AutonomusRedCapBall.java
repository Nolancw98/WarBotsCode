package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Anthony on 11/2/2016.
 */

@Autonomous(name="Red Cap Ball", group="Cool")
public class AutonomusRedCapBall extends LinearOpMode
{
    static public Hardware10415 robot;
    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() throws InterruptedException
    {
        robot = new Hardware10415();
        robot.init(hardwareMap);

        //reset encoders
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Help me");
        idle();

        //Wait for play button
        waitForStart();


        drive(7.5);

        setPower(0);

        telemetry.addLine("Autonomus over");



    }

    public void drive(double inches) throws InterruptedException
    {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        runUsingEncoders();

        robot.motorFrontLeft.setTargetPosition(newLeftTarget);
        robot.motorFrontRight.setTargetPosition(newRightTarget);
        robot.motorBackLeft.setTargetPosition(newLeftTarget);
        robot.motorBackRight.setTargetPosition(newRightTarget);

        setPower(50);

        while(robot.motorFrontLeft.isBusy())
        {
            telemetry.addLine("Moving to pos 1");
            //idle();
        }

        setPower(0);
        resetEncoders();
    }

    public void setPower(int power)
    {
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

    public void runUsingEncoders()
    {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
