package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRelic
{
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor wheelOne = null;
    public DcMotor wheelTwo = null;
    public DcMotor wheelThree = null;
    public DcMotor wheelFour = null;
    public DcMotor CubeLift1 = null;
    public DcMotor CubeLift2 = null;

    public Servo extender = null;
    public CRServo relicClawLifter = null;
    public Servo clawExtenderRight, clawExtenderLeft = null;

    public CRServo    CubeTimeBelt= null;

    public Servo RelClaw = null;
    public TouchSensor CubeSlideSwitchR = null;
    public TouchSensor CubeSlideSwitchL = null;
    public TouchSensor CubeLiftSwitch = null;



    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRelic(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //  telemetry.addData("Status", "Initialized");


        wheelOne = hwMap.dcMotor.get("wheelOne");
        wheelTwo = hwMap.dcMotor.get("wheelTwo");
        wheelThree = hwMap.dcMotor.get("wheelThree");
        wheelFour = hwMap.dcMotor.get("wheelFour");

        CubeTimeBelt= hwMap.crservo.get("CubeTimeBelt");

        wheelOne.setDirection(DcMotor.Direction.FORWARD);//
        wheelTwo.setDirection(DcMotor.Direction.FORWARD); //

        wheelThree.setDirection(DcMotor.Direction.FORWARD);//
        wheelFour.setDirection(DcMotor.Direction.FORWARD); //


        wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RelClaw= hwMap.get(Servo.class, "RelClaw");
        extender = hwMap.get(Servo.class, "gemExtender");
        relicClawLifter = hwMap.crservo.get("relicClawLifter");
        clawExtenderLeft = hwMap.get(Servo.class, "clawExtenderLeft");
        clawExtenderRight = hwMap.get(Servo.class, "clawExtenderRight");
        CubeSlideSwitchL = hwMap.get(TouchSensor.class,"CubeSlideSwitchL");
        CubeSlideSwitchR = hwMap.get(TouchSensor.class,"CubeSlideSwitchR");
        //CubeLiftSwitch = hwMap.get(TouchSensor.class,"CubeLiftSwitch");
        //CubeLift1 = hwMap.get(DcMotor.class,"CubeLift1");
        //CubeLift2 = hwMap.get(DcMotor.class,"CubeLift2");


        relicClawLifter.setPower(0);
        CubeTimeBelt.setPower(0);
        extender.setPosition(0.0);
        RelClaw.setPosition(0.0);
        clawExtenderLeft.setPosition(0.0);
        clawExtenderRight.setPosition(1);


        stopRobot();

    }



    public void setDriveEncoderMode( Boolean setOrStop){
        if(setOrStop) {
            wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            wheelOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelFour.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    public void setDriveNonEncoderMode( ){

        wheelOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stopRobot () {
        wheelOne.setPower(0);
        wheelTwo.setPower(0);
        wheelThree.setPower(0);
        wheelFour.setPower(0);
    }



    public void setDriveSpeed(double leftSpeed,double rightSpeed, int driveDirection){
        wheelOne.setPower(leftSpeed*driveDirection);
        wheelTwo.setPower(leftSpeed*driveDirection);
        wheelThree.setPower(rightSpeed*driveDirection);
        wheelFour.setPower(rightSpeed*driveDirection);

    }


    public double getAngle(double x, double y){
        double hypotneus;
        double angle = 0;

        hypotneus = Math.sqrt(x*x + y*y);

        y = -y;

        if (x>0 && y>0){
            angle = Math.toDegrees(Math.asin(x/hypotneus));
        } else if (x>0 && y<0){
            angle = Math.toDegrees(Math.asin(Math.abs(y)/hypotneus)) + 90;
        } else if (x < 0 && y < 0){
            angle = Math.toDegrees(Math.asin(Math.abs(x)/hypotneus)) + 180;
        } else if (x < 0 && y > 0){
            angle = Math.toDegrees(Math.asin(y/hypotneus)) + 270;
        } else if (x == 0) {
            if (y > 0)
                angle = 0;
            else if (y < 0)
                angle = 180;
        }else if (y == 0) {
            if (x > 0)
                angle = 90;
            else if (x < 0)
                angle = 270;
        }else
            angle = -1;

        return angle;
    }

    // This program simply drives the robot forward with an specified angle
    public void angleDrive (double speed, double angle, double rotation){

        double v1, v2;

        if (Math.abs(angle) % 90 < Math.abs((angle % 90) - 45)){
            v1 = speed;
            v2 = -speed;
        } else {
            v1 = speed;
            v2 = 0;
        }

        v1/=2;
        v2/=2;

        //the motors are configured to go counterclockwise if the power are all set to 1
        v1 = -v1;
        v2 = -v2;

        rotation/=-2;

        if (angle >= 0 && angle <90)
            setSpeed(v1 + rotation, v2 + rotation, -v1 + rotation, -v2 + rotation);
        else if (angle >= 90 && angle < 180)
            setSpeed(-v2 + rotation, v1 + rotation, v2 + rotation, -v1 + rotation);
        else if (angle >= 180 && angle < 270)
            setSpeed(-v1 + rotation, -v2 + rotation, v1 + rotation, v2 + rotation);
        else if (angle >= 270 && angle < 360)
            setSpeed(v2 + rotation, -v1 + rotation, -v2 + rotation, v1 + rotation);
        else
            setSpeed(0, 0, 0, 0);

    }





    public void setSpeed(double one, double two, double three, double four) {

        wheelOne.setPower(one);
        wheelTwo.setPower(two);
        wheelThree.setPower(three);
        wheelFour.setPower(four);
    }


    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}