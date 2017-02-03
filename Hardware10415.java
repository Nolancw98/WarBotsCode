package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Anthony on 10/23/2016.
 */
public class Hardware10415
{
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorLauncherLeft;
    public DcMotor motorLauncherRight;
    public DcMotor motorLift;
    public DcMotor motorButtonPusher;

    public Servo button;

    HardwareMap hwMap;



    public Hardware10415()
    {

    }

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");
        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorLauncherLeft = hwMap.dcMotor.get("launcherLeft");
        motorLauncherRight = hwMap.dcMotor.get("launcherRight");
        motorLift = hwMap.dcMotor.get("lift");
        motorButtonPusher = hwMap.dcMotor.get("buttonPusher");


        //Servos
        //button = hwMap.servo.get("button");

        //Setting Direction
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLauncherLeft.setDirection(DcMotor.Direction.REVERSE);
    }
}
