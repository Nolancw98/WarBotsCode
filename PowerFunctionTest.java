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
@TeleOp(name="PowerFunctionTest", group="Cool")
public class PowerFunctionTest extends OpMode {
    Hardware10415 robot = new Hardware10415();

    double launcherPower = 1;
    boolean launcherPadUp = false;
    boolean launcherPadDown =  false;
    //int speedPID = 4000;
    double lPower = 1;
    boolean newVoltage = true;
    double voltage = 0.0;
    final double launchSlope = -0.186;
    final double launchYInt = 3.38;
    double motorPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.motorLauncherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorLauncherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop()
    {
        if(gamepad1.a)
        {
            if(newVoltage)
            {
                voltage = hardwareMap.voltageSensor.get("back").getVoltage();
                newVoltage = false;
            }
            motorPower = getScalar(voltage);
            robot.motorLauncherLeft.setPower(motorPower);
            robot.motorLauncherRight.setPower(motorPower);

        }
        else
        {
            robot.motorLauncherLeft.setPower(0);
            robot.motorLauncherRight.setPower(0);
        }

        if(gamepad1.b && !gamepad1.a)
        {
            newVoltage = true;
        }
        telemetry.addData("newVoltage", newVoltage);
        telemetry.addData("motorPower", motorPower);
        robot.motorLift.setPower(gamepad1.left_stick_y);
    }

    public void stop()
    {

    }

    public double getScalar(double voltage)
    {
        return (voltage*launchSlope) + launchYInt;
    }
}
