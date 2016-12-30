package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SensorTest", group = "Cool")

public class SensorTester extends OpMode{
    Hardware10415 robot = new Hardware10415();
    ModernRoboticsI2cGyro gyro = null;
    ModernRoboticsI2cColorSensor colorSensor = null;
    ModernRoboticsAnalogOpticalDistanceSensor lineSensor = null;

    public void init()
    {
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        lineSensor = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("lineSensor");
    }

    public void loop()
    {
        telemetry.addData("Color", colorSensor.red());
        telemetry.addData("ODS" , lineSensor.getRawLightDetected());
    }

    public void stop(){}
}
