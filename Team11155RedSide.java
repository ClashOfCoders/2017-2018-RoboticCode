package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;
import org.firstinspires.ftc.teamcode.HardwareRelic;


/**
 * Created by jianqiuzhang on 11/29/17.
 */


@Autonomous(name = "Team11155RedSide", group = "Auto")
//@Disabled                            // Comment this out to add to the opmode list
public class Team11155RedSide extends LinearOpMode {
    HardwareRelic robot = new HardwareRelic();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        robot.gyro.calibrate();
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        waitForStart();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;


        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        robot.cdim.setDigitalChannelMode(robot.LED_CHANNEL, DigitalChannel.Mode.OUTPUT);


        // turn the LED on in the beginning, just so user will know that the sensor is active.
        robot.cdim.setDigitalChannelState(robot.LED_CHANNEL, bLedOn);




        robot.GemRotater.setPosition(robot.GemRotatorInitialPosition);
        robot.GemExtender.setPosition(robot.GemExtenderLastPosition);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        int redFirst = robot.sensorRGB.red();
        int blueFirst = robot.sensorRGB.blue();


        robot.GemExtender.setPosition(robot.GemExtenderMiddlePosition);

        robot.GemRotater.setPosition(robot.GemRotatorMidPosition);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        robot.GemExtender.setPosition(robot.GemExtenderLastPosition-0.1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        int redSecond = robot.sensorRGB.red();
        int blueSecond = robot.sensorRGB.blue();

        // between the two readings, use the one with larger difference between blue and red
        if(Math.abs(redFirst-blueFirst)>Math.abs((redSecond-blueSecond))){
            //Use the first set of readings
            if(redFirst>blueFirst){
                // The first ball is a red
                robot.GemRotater.setPosition(robot.GemRotatorLastPosition);

            }
            else
                robot.GemRotater.setPosition(robot.GemRotatorInitialPosition);


        }
        else {
            if (blueSecond < redSecond) {

                robot.GemRotater.setPosition(robot.GemRotatorInitialPosition);

            }
            else

                robot.GemRotater.setPosition(robot.GemRotatorLastPosition);



        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        robot.GemExtender.setPosition(robot.GemExtenderMiddlePosition);
        robot.GemRotater.setPosition(robot.GemRotatorInitialPosition);
        robot.GemExtender.setPosition(robot.GemExtenderInitialPosition);



        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        double startTime=getRuntime();
        while (opModeIsActive() && (getRuntime()-startTime)<2) {
            robot.angleDrive(this,0.5,180,0);
        }
        robot.stopRobot();
        double gyroTarget=robot.gyroAngle(robot.gyro.getIntegratedZValue());
        while (opModeIsActive() && (robot.gyroAngle(robot.gyro.getIntegratedZValue())-gyroTarget)<90){
            robot.angleDrive(this,0,0,0.5);
        }
    }
}
