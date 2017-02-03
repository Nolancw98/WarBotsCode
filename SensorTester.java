package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
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
    ModernRoboticsDigitalTouchSensor touchSensor = null;

    public void init()
    {
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("colorSensor");
        lineSensor = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("lineSensor");
        colorSensor.resetDeviceConfigurationForOpMode();
        colorSensor.enableLed(false);

        touchSensor = (ModernRoboticsDigitalTouchSensor) hardwareMap.touchSensor.get("touch");
    }

    public void loop()
    {

        telemetry.addData("Color", colorSensor.argb());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("ODS" , lineSensor.getLightDetected());
        telemetry.addData("Gyro",gyro.getIntegratedZValue());
        telemetry.addData("Touch",touchSensor.isPressed());
    }

    public void stop(){}
}
