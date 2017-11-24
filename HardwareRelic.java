package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    public DcMotor wheelOne = null;
    public DcMotor wheelTwo = null;
    public DcMotor wheelThree = null;
    public DcMotor wheelFour = null;

    public DcMotor dcBallCollector = null;
    public DcMotor capBallLifter = null;

    public Servo ballFeeder = null;
    public Servo ballLifterLeft = null;
    public Servo ballLifterRight = null;
    public Servo beaconButtonPusher=null;
    public Servo capBallClaw=null;
    // private Servo ballCollector = null;

    public ModernRoboticsI2cGyro gyro    = null;
    byte[] rangeAcache;
    byte[] rangeCcache;

    public I2cDevice rangeA;
    public I2cDevice rangeC;
    public I2cDeviceSynchImpl rangeAreader;
    public I2cDeviceSynchImpl rangeCreader;


    // public ModernRoboticsI2cRangeSensor  range = null;// Additional Gyro device
    // public ModernRoboticsI2cRangeSensor  rangeFront = null;// Additional Gyro device
    public OpticalDistanceSensor odsSensor;
    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public TouchSensor touchSensor;

    static final int LED_CHANNEL = 5;


    public double     wheelDistance = 16;          // center to center wheel distance;
    public double     robotWidth =16.25;
    public double    robotLength = 17.75;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.78 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public int  max_encoder_clicks_per_second =(int) (6600/60*28*0.75);

    public static final double CONTINUOUS_SERVO_stop = 0.52;
    public static final double CONTINUOUS_SERVO_FORWARD = 1;
    public static final double CONTINUOUS_SERVO_REVERSE = 0;
    public static final double Servo_start_position = 0;
    public static final double Servo_hold_position = 0.6;
    public static final double Servo_max_position = 1;
    public static final double Servo_min_position = 0.4;
    public static final double Hitech_Servo_Stop_Left =1;
    public static final double Hitech_Servo_Stop_Right=0;
    double DRIVE_SPEED = 0.15;     // Nominal speed for better accuracy.
    double TURN_SPEED = 0.15;     // Nominal half speed for better accuracy.
    double stopDistance=9;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.001;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.003;     // Larger is more responsive, but also less stable


    public int leftMaxCount=2000;
    public int rightMaxCount=2000;
    /* Public OpMode members. */
    public int targetCountPerSecondBallShooter=1575;
    public int targetCountPerSecondBallShooterHigh=1800;

    /* local OpMode members. */
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

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hwMap.dcMotor.get("LeftMotor");
        rightMotor = hwMap.dcMotor.get("RightMotor");

        wheelOne = hwMap.dcMotor.get("wheelOne");
        wheelTwo = hwMap.dcMotor.get("wheelTwo");
        wheelThree = hwMap.dcMotor.get("wheelThree");
        wheelFour = hwMap.dcMotor.get("wheelFour");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        wheelOne.setDirection(DcMotor.Direction.FORWARD);//
        wheelTwo.setDirection(DcMotor.Direction.FORWARD); //

        wheelThree.setDirection(DcMotor.Direction.FORWARD);//
        wheelFour.setDirection(DcMotor.Direction.FORWARD); //




/*
        // run the motor for 1 seconds
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        leftMaxCount=leftMotor.getCurrentPosition();
        rightMaxCount=rightMotor.getCurrentPosition();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        max_encoder_clicks_per_second=Math.min(leftMaxCount,rightMaxCount);

        if(max_encoder_clicks_per_second>0){
            leftMotor.setMaxSpeed(max_encoder_clicks_per_second);
            rightMotor.setMaxSpeed(max_encoder_clicks_per_second);}
        else{

        }

        if(leftMaxCount>0 && rightMaxCount>0 && Math.min(leftMaxCount,rightMaxCount)>2000){
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetCountPerSecondBallShooter=1400;

        }
        else if (Math.min(leftMaxCount,rightMaxCount)>1600 && leftMaxCount>0 && rightMaxCount>0){
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetCountPerSecondBallShooter=1410;
        }
        else if (Math.min(leftMaxCount,rightMaxCount)>1400 && leftMaxCount>0 && rightMaxCount>0){
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetCountPerSecondBallShooter=max_encoder_clicks_per_second;
        }
        else if (leftMaxCount==0 || rightMaxCount==0){
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //    leftMotor.setMaxSpeed(2000);
            //   rightMotor.setMaxSpeed(2000);
            max_encoder_clicks_per_second=2000;
            targetCountPerSecondBallShooter=1100;

        }



        // Set all motors to run with encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.








*/
        wheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopRobot();


        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        // range = (ModernRoboticsI2cRangeSensor)hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //  rangeFront= (ModernRoboticsI2cRangeSensor)hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeFront");
        odsSensor = hwMap.opticalDistanceSensor.get("sensor_ods");
        // get a reference to our DeviceInterfaceModule object.

        cdim = hwMap.deviceInterfaceModule.get("dim");
        sensorRGB = hwMap.colorSensor.get("sensor_color");
        touchSensor = hwMap.touchSensor.get("sensor_touch");

        rangeA = hwMap.i2cDevice.get("range28");
        rangeC = hwMap.i2cDevice.get("range2a");

        rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);
        rangeCreader = new I2cDeviceSynchImpl(rangeC, I2cAddr.create8bit(0x2a), false);

        rangeAreader.engage();
        rangeCreader.engage();




        /* code for dealing with the color sensors

         */

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        // turn the LED on in the beginning for a second, just so user will know that the sensor is active.
        // then it is turned off
        // cdim.setDigitalChannelState(LED_CHANNEL, true);

        cdim.setDigitalChannelState(LED_CHANNEL, false);
        stopRobot();
    }

    public double rangeRead(I2cDeviceSynchImpl rangeReader){
        byte[] rangeCache;
        rangeCache=rangeReader.read(0x04, 2);
        double rangeDistance=(double)(rangeCache[0] & 0xFF)/2.54;
        return rangeDistance;
    }
    public void shootBall(double mpower, long waitMS) {
        leftMotor.setPower(mpower);
        rightMotor.setPower(mpower);

        try {
            Thread.sleep(waitMS);

        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        ballFeeder.setPosition(CONTINUOUS_SERVO_FORWARD);
        try {
            Thread.sleep(waitMS);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        ballFeeder.setPosition(CONTINUOUS_SERVO_REVERSE);
        try {
            Thread.sleep(waitMS);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        ballFeeder.setPosition(CONTINUOUS_SERVO_stop);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

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


    public void stopEffectors(){
        capBallLifter.setPower(0);
        dcBallCollector.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        ballFeeder.setPosition(CONTINUOUS_SERVO_stop);
        beaconButtonPusher.setPosition(CONTINUOUS_SERVO_stop);
        ballLifterLeft.setPosition(Hitech_Servo_Stop_Left);
        ballLifterRight.setPosition(Hitech_Servo_Stop_Right);
        capBallClaw.setPosition(Servo_start_position);
    }

    public void setDriveSpeed(double leftSpeed,double rightSpeed, int driveDirection){
        wheelOne.setPower(leftSpeed*driveDirection);
        wheelTwo.setPower(leftSpeed*driveDirection);
        wheelThree.setPower(rightSpeed*driveDirection);
        wheelFour.setPower(rightSpeed*driveDirection);

    }

    /*
    public void  pushBeaconButton(long timeDifferenceMS, LinearOpMode currentOpMode, I2cDeviceSynchImpl rangeReader){

        double distToWall = rangeRead(rangeReader);
        long timeToPush = (long) (distToWall - 3) * 600 - timeDifferenceMS + 700;
        timeToPush=(long) Range.clip(timeToPush,1,2500);
        ElapsedTime runtime =new ElapsedTime();
        runtime.reset();
        if (currentOpMode.isStopRequested()) {
            return;
        }
        while(currentOpMode.opModeIsActive() && runtime.milliseconds()<timeToPush) {

            beaconButtonPusher.setPosition(CONTINUOUS_SERVO_FORWARD);
            currentOpMode.idle();
        }
        runtime.reset();
        while(currentOpMode.opModeIsActive() && runtime.milliseconds()<timeToPush) {

            beaconButtonPusher.setPosition(CONTINUOUS_SERVO_REVERSE);
            currentOpMode.idle();
        }

        beaconButtonPusher.setPosition(CONTINUOUS_SERVO_stop);
    }


    public void  outBeaconButton(long timeDifferenceMS,LinearOpMode currentOpMode ){
        ElapsedTime runtime =new ElapsedTime();
        runtime.reset();

        while(currentOpMode.opModeIsActive() && runtime.milliseconds()<timeDifferenceMS) {

            beaconButtonPusher.setPosition(CONTINUOUS_SERVO_FORWARD);
            currentOpMode.idle();
        }
        beaconButtonPusher.setPosition(CONTINUOUS_SERVO_stop);


    }
    public void  retractBeaconButton(long timeDifferenceMS,LinearOpMode currentOpMode ){

        ElapsedTime runtime =new ElapsedTime();
        runtime.reset();

        while(currentOpMode.opModeIsActive() && runtime.milliseconds()<timeDifferenceMS) {

            beaconButtonPusher.setPosition(CONTINUOUS_SERVO_REVERSE);
            currentOpMode.idle();
        }
        beaconButtonPusher.setPosition(CONTINUOUS_SERVO_stop);
    }

    */


    public double getAngle(double x, double y){
        double hypotneus;
        double angle = 0;

        hypotneus = Math.sqrt(x*x + y*y);

        if (x>0 && y>0){
            angle = Math.toDegrees(Math.asin(x/hypotneus));
        } else if (x>0 && y<0){
            angle = Math.toDegrees(Math.asin(Math.abs(y)/hypotneus)) + 90;
        } else if (x < 0 && y < 0){
            angle = Math.toDegrees(Math.asin(Math.abs(x)/hypotneus)) + 180;
        } else if (x < 0 && y > 0){
            angle = Math.toDegrees(Math.asin(y/hypotneus)) + 270;
        } else if (x == 0)
            if (y > 0)
                angle = 0;
            else
                angle = 180;
        else if (y == 0)
            if (x > 0)
                angle = 90;
            else
                angle = 270;

        angleDrive(hypotneus, angle);

        return angle;
    }

    // This program simply drives the robot forward with an specified angle
    public void angleDrive (double speed, double angle){

        double v1, v2;

        if (angle % 90 < Math.abs((angle % 90) - 45)){
            v1 = v2 = speed;
        } else {
            v1 = speed;
            v2 = 0;
        }

        v1/=2;
        v2/=2;

        if (angle >= 0 && angle <90)
            setSpeed(v1, v2, -v1, -v2);
        else if (angle >= 90 && angle < 180)
            setSpeed(v2, v1, -v2, v1);
        else if (angle >= 180 && angle < 270)
            setSpeed(-v1, -v2, v1, v2);
        else if (angle >= 270 && angle < 360)
            setSpeed(-v2, -v1, v2, v1);

    }



    /*
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */


    public void setSpeed(double one, double two, double three, double four) {

        wheelOne.setPower(one);
        wheelTwo.setPower(two);
        wheelThree.setPower(three);
        wheelFour.setPower(four);
    }
    public void gyroDrive(double currentAngle, double targetAngle){


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
