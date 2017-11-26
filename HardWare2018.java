package org.firstinspires.ftc.teamcode;

/**
 * Created by jianqiuzhang on 11/10/17.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardWare2018 {
    public DcMotor  WheelFrontLeft   = null;
    public DcMotor  WheelFrontRight  = null;
    public DcMotor  WheelBackLeft    = null;
    public DcMotor  WheelBackRight   = null;
    public DcMotor CubeLift1 = null;
    public DcMotor CubeLift2 = null;
    public Servo    RelicLeft        = null;
    public Servo    RelicRight       = null;
    public Servo    RelicClaw        = null;
    public Servo    RelicLift        = null;
    public Servo    CubeClawLeft    = null;
    public Servo    CubeClawRight   =null;
    public Servo    CubeLiftArm =null;
    public Servo    CubeTimeBelt= null;
    public TouchSensor CubeSlideSwitchR = null;
    public TouchSensor CubeSlideSwitchL = null;
    public TouchSensor CubeLiftSwitch = null;

    public ElapsedTime getPeriod() {
        return period;
    }

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardWare2018(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        CubeSlideSwitchL = hwMap.get(TouchSensor.class,"CubeSlideSwitchL");
        CubeSlideSwitchR = hwMap.get(TouchSensor.class,"CubeSlideSwitchR");
        WheelFrontLeft  = hwMap.get(DcMotor.class, "WheelFrontLeft");
        WheelFrontRight = hwMap.get(DcMotor.class, "WheelFrontRight");
        WheelBackLeft  = hwMap.get(DcMotor.class, "WheelBackLeft");
        WheelBackRight = hwMap.get(DcMotor.class, "WheelBackRight");
        CubeLift1 = hwMap.get(DcMotor.class, "CubeLift1");
        CubeLift2 = hwMap.get(DcMotor.class, "CubeLift2");
        CubeLiftSwitch = hwMap.get(TouchSensor.class,"CubeLiftSwitch");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        WheelFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        WheelFrontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        WheelBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        WheelBackRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        CubeLift1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        CubeLift2.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);
        CubeLift1.setPower(0);
        CubeLift2.setPower(0);
        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     //   leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

        RelicLeft  = hwMap.get(Servo.class, "RelicLeft");
        RelicRight = hwMap.get(Servo.class, "RelicRight");
        RelicClaw  = hwMap.get(Servo.class, "RelicClaw");
        RelicLift = hwMap.get(Servo.class, "RelicLift");
        RelicLeft.setPosition(0.9);
        RelicRight.setPosition(0.1);
        RelicClaw.setPosition(0);
        RelicLift.setPosition(0.52);

        CubeClawLeft  = hwMap.get(Servo.class, "CubeClawLeft");
        CubeClawRight = hwMap.get(Servo.class, "CubeClawRight");
        CubeLiftArm  = hwMap.get(Servo.class, "CubeLiftArm");
        CubeTimeBelt= hwMap.get(Servo.class, "CubeTimeBelt");
        CubeClawLeft.setPosition(0);
        CubeClawRight.setPosition(1);
        CubeLiftArm.setPosition(0);
        CubeTimeBelt.setPosition(0.5);


    }
    public void stopRobot(){
        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);

    }
}
