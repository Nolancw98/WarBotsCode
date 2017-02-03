package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Anthony on 9/14/2016.
 */
@TeleOp(name="tOpNonPID", group="Cool")
public class tOpNonPID extends OpMode {
    Hardware10415 robot = new Hardware10415();

    double launcherPower = 1;
    boolean launcherPadUp = false;
    boolean launcherPadDown =  false;
    //int speedPID = 4000;
    double lPower = 1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.motorLauncherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorLauncherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        double left = scaleInput(-gamepad1.left_stick_y);
        double right = scaleInput(-gamepad1.right_stick_y);

        if(gamepad1.left_trigger > .5)
        {
            left /= 2;
            right /= 2;
        }


        //Player 2 controls
        if(gamepad2.a)
        {
            robot.motorLauncherRight.setPower(lPower);
            robot.motorLauncherLeft.setPower(lPower);

        }
        else {
            robot.motorLauncherRight.setPower(0);
            robot.motorLauncherLeft.setPower(0);

        }

        if(gamepad2.dpad_up && !launcherPadUp) {
            if(lPower < 1)
            {
                lPower += .05;
            }
                        /*if(launcherPower < 1.0) {
                                launcherPower += .1;
                                if(launcherPower > 1.0)
                                {
                                        launcherPower = 1.0;
                                }
                        }
                        */
            //telemetry.addLine("Launcher Speed: " + launcherPower);
        }
        else if(gamepad2.dpad_down && !launcherPadDown)
        {
            if(lPower > 0)
            {
                lPower -= .05;
            }
                        /*if(launcherPower < 0.1) {
                                launcherPower -= .1;
                                if(launcherPower < 0)
                                {
                                        launcherPower = .1;
                                }
                        }*/
            //telemetry.addLine("Launcher Speed: " + launcherPower);
        }
        telemetry.addLine("Launcher Speed: " + lPower);
        //telemetry.addLine("Launcher MaxSpeed:" + speedPID);
        launcherPadUp = gamepad2.dpad_up;
        launcherPadDown = gamepad2.dpad_down;

        //if(gamepad2.y){
        //       runCatapult();
        //}
                /*
                if(gamepad2.left_trigger > 0.0)
                {
                        robot.motorLift.setPower(gamepad2.left_stick_y);
                        telemetry.addLine("Test");
                }
                else if(gamepad2.right_trigger > 0)
                {
                        robot.motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
                        robot.motorLift.setPower(gamepad2.left_stick_y);
                }

                else {
                        robot.motorLift.setPower(0);
                }
                */
        // write values to wheel motor
        robot.motorLift.setPower(gamepad2.left_stick_y);
        robot.motorFrontRight.setPower(right);
        robot.motorFrontLeft.setPower(left);
        robot.motorBackRight.setPower(right);
        robot.motorBackLeft.setPower(left);
    }


    @Override
    public void stop(){

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

        /*void runCatapult(){
                if(catapultRunning == false *//*|| robot.motorCatapult.isBusy() == false*//*)
                {
                        catapultRunning = true;
                        telemetry.addLine("True");
                        robot.motorCatapult.setPower(20);
                        robot.motorCatapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorCatapult.setTargetPosition(350*4);
                        while(robot.motorCatapult.isBusy())
                        {
                                //Nothing
                        }
                        robot.motorCatapult.setTargetPosition(5*4); //Move to initl pos
                        while(robot.motorCatapult.isBusy())
                        {
                                //Nothing
                        }
                        robot.motorCatapult.setPower(0);
                        robot.motorCatapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        catapultRunning = false;
                        telemetry.addLine("false");
                }
        }*/
}

