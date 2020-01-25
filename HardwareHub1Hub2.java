package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.RevBulkData;

/**
 * Hardware class for the FULL ROBOT (hub1 & hub2)
 */
public class HardwareHub1Hub2
{
    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    public BNO055IMU      imu      = null;

    //====== MECANUM DRIVETRAIN MOTORS (no encoder cabling!) =====
    public DcMotor frontRightMotor       = null;
    public int     frontRightMotorPos    = 0;        // current encoder count
    public int     frontRightMotorVel    = 0;        // encoder counts per second
    private double frontRightLast        = 0.0;      // Track current motor power so we can limit abrupt changes

    public DcMotor backRightMotor        = null;
    public int     backRightMotorPos     = 0;        // current encoder count
    public int     backRightMotorVel     = 0;        // encoder counts per second
    private double backRightLast         = 0.0;
    public int     frontLeftMotorPos     = 0;        // current encoder count

    public DcMotor frontLeftMotor        = null;
    public int     frontLeftMotorVel     = 0;        // encoder counts per second
    private double frontLeftLast         = 0.0;

    public DcMotor backLeftMotor         = null;
    public int     backLeftMotorPos      = 0;        // current encoder count
    public int     backLeftMotorVel      = 0;        // encoder counts per second
    private double backLeftLast          = 0.0;

    //====== ODOMETRY ENCODERS (encoder values only!) =====
    public DcMotor rightOdometer      = null;
    public int     rightOdometerCount = 0;           // current encoder count
    public int     rightOdometerPrev  = 0;           // previous encoder count

    public DcMotor leftOdometer       = null;
    public int     leftOdometerCount  = 0;           // current encoder count
    public int     leftOdometerPrev   = 0;           // previous encoder count

    public DcMotor rearOdometer       = null;
    public int     rearOdometerCount  = 0;           // current encoder count
    public int     rearOdometerPrev   = 0;           // previous encoder count

    //The amount of US Digital E8T encoder ticks for each inch the robot moves.
    public double COUNTS_PER_INCH = 611.115;   // 2880 pulse/rev (720 cycles/rev); 1.5" diameter omni wheel = 4.712" circumference.

//    //====== FOUNDATION REPOSITIONING FINGER SERVOS =====
//    public Servo   fingerServoR = null;
//    public Servo   fingerServoL = null;
//
//    public double  RIGHT_FINGER_RAISED  = 0.75;
//    public double  RIGHT_FINGER_LOWERED = 0.17;
//    public double  LEFT_FINGER_RAISED   = 0.25;
//    public double  LEFT_FINGER_LOWERED  = 0.85;
//
//    //====== REV COLOR DISTANCE SENSOR (V2)=====
//    public ColorSensor sensorColor = null;
//
//    //====== SCISSOR-LIFT MOTORS & MAGNETIC LIMIT SWITCHES =====
//    public DcMotor        scissorMotorR         = null;
//    public DcMotor        scissorMotorL         = null;
//    public int            scissorMotorPosMin    = 1;
//    public int            scissorMotorPosR      = 0;
//    public int            scissorMotorVelR      = 0;        // encoder counts per second
//    public int            scissorMotorPosL      = 0;
//    public int            scissorMotorVelL      = 0;        // encoder counts per second
//    public int            scissorMotorTargetR   = 0;        // target encoder count (right scissor)
//    public int            scissorMotorTargetL   = 0;        // target encoder count (left scissor)
//    public boolean        scissorMotorAutoR     = false;    // true=RUN_TO_POSITION; false=RUN_USING_ENCODER
//    public boolean        scissorMotorAutoL     = false;    // true=RUN_TO_POSITION; false=RUN_USING_ENCODER
//
//    //  public DigitalChannel scissorMotorLimitR    = null;    // Not used; See h2bulkData
////  public DigitalChannel scissorMotorLimitL    = null;    // Not used; See h2bulkData
//    public boolean        scissorMotorLimitL_on = false;
//    public boolean        scissorMotorLimitR_on = false;
//
//    //====== Cascade MOTOR =====
//    public DcMotor        cascadeMotor       = null;
//    public int            cascadeMotorPos    = 0;        // current encoder count
//    public int            cascadeMotorVel    = 0;        // encoder counts per second
//    public int            cascadeMotorTarget = 0;        // target encoder count
//    public boolean        cascadeMotorAuto   = false;    // true=RUN_TO_POSITION; false=RUN_USING_ENCODER
//
//    //====== STONE-GRABBER SERVOS =====
//    public Servo          grabberServoR = null;
//    public Servo          grabberServoL = null;
//
//    public double RIGHT_GRABBER_OPEN   = 0.40;
//    public double RIGHT_GRABBER_UP     = RIGHT_GRABBER_OPEN - 0.20;
//    public double RIGHT_GRABBER_CLOSED = RIGHT_GRABBER_OPEN + 0.27;  // 0.0 = up; 1.0 = down
//    public double RIGHT_GRABBER_DOWN   = RIGHT_GRABBER_OPEN + 0.40;
//
//    public double LEFT_GRABBER_OPEN    = 0.58;
//    public double LEFT_GRABBER_UP      = LEFT_GRABBER_OPEN  + 0.20;
//    public double LEFT_GRABBER_CLOSED  = LEFT_GRABBER_OPEN  - 0.27;  // 1.0 = up; 0.0 = down
//    public double LEFT_GRABBER_DOWN    = LEFT_GRABBER_OPEN  - 0.40;

    //====== REV EXPANSION HUB =====
//    ExpansionHubEx    expansionHub1, expansionHub2;
//    RevBulkData       h1bulkData, h2bulkData;
    // Motor status can be accessed by PORT/MOTOR NUMBER (as well as by OBJECT)
    int               HUB_MOTOR0 = 0;  // Motor Port #0
    int               HUB_MOTOR1 = 1;  // Motor Port #1
    int               HUB_MOTOR2 = 2;  // Motor Port #2
    int               HUB_MOTOR3 = 3;  // Motor Port #3
    // Digital input status can be access by PIN NUMBER (as well as by OBJECT)
    int               HUB_DIGPIN0 = 0; // Digital I/O input pin #0
    int               HUB_DIGPIN2 = 2; // Digital I/O input pin #2

    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareHub1Hub2(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

//        fingerServoR = hwMap.servo.get("right_finger");   // servo port 0
//        fingerServoL = hwMap.servo.get("left_finger");    // servo port 1

        // Define and Initialize drivetrain motors
        frontRightMotor = hwMap.dcMotor.get("FrontRight"); // port 0 (forward)
        backRightMotor  = hwMap.dcMotor.get("RearRight");  // port 1 (forward)
        frontLeftMotor  = hwMap.dcMotor.get("FrontLeft");  // port 2 (REVERSE)
        backLeftMotor   = hwMap.dcMotor.get("RearLeft");   // port 3 (REVERSE)

        rightOdometer   = hwMap.dcMotor.get("FrontRight");  // port0 (must be a "forward" motor port)
        leftOdometer    = hwMap.dcMotor.get("FrontLeft");   // port2 (must be a "REVERSE" motor port)
        rearOdometer    = hwMap.dcMotor.get("RearLeft");    // port3 (must be a "REVERSE" motor port)

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  // goBilda fwd/rev opposite of Matrix motors!
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all drivetrain motors to zero power
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);

        // Set all drivetrain motors to run without encoders.
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero all odometry encoders
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        //Initialize Foundation re-position servo fingers
//        fingerServoR.setPosition( RIGHT_FINGER_RAISED );
//        fingerServoL.setPosition( LEFT_FINGER_RAISED );
//
//        //Instantiate REV color distance sensor (V2)
//        sensorColor = hwMap.get(ColorSensor.class, "ColorSensor");

//        //Configure scissor-lift motors
//        scissorMotorR = hwMap.dcMotor.get("ScissorRight");
//        scissorMotorL = hwMap.dcMotor.get("ScissorLeft");
//        scissorMotorR.setDirection(DcMotor.Direction.FORWARD);  // +1.0 power raises; -1.0 lowers
//        scissorMotorL.setDirection(DcMotor.Direction.FORWARD);
//        scissorMotorR.setPower(0);
//        scissorMotorL.setPower(0);
//        scissorMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // ensure starting count is 0
//        scissorMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        scissorMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // ensure starting count is 0
//        scissorMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        scissorMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        scissorMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Instantiate scissor-lift magnetic range sensors
//      scissorMotorLimitR = hwMap.digitalChannel.get("ScissorLimit2");   // both limit2 & limit3 report RIGHT status
//      scissorMotorLimitL = hwMap.digitalChannel.get("ScissorLimit0");   // both limit0 & limit1 report LEFT status

        //Cascade Motor
//        cascadeMotor = hwMap.dcMotor.get("CascadeMotor");
//        cascadeMotor.setDirection(DcMotor.Direction.REVERSE);
//        cascadeMotor.setPower(0);
//        cascadeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // ensure starting count is 0
//        cascadeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        cascadeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //Initialize stone-grabber servos
//        grabberServoR = hwMap.servo.get("right_grabber");   // servo port 0
//        grabberServoL = hwMap.servo.get("left_grabber");    // servo port 1
//        grabberServoR.setPosition( RIGHT_GRABBER_UP );
//        grabberServoL.setPosition( LEFT_GRABBER_UP );
//
//        // REV Expansion Hub objects used to perform bulkData transfers
//        expansionHub1 = hwMap.get(ExpansionHubEx.class, "Main Hub");
//        expansionHub2 = hwMap.get(ExpansionHubEx.class, "Secondary Hub");

        // Initialize REV Expansion Hub IMU
        initIMU();
    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055imu_DQ2A8Q3S.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        double heading = -(double)angles.firstAngle;
        return heading;  // 90 is CW; -90 is CCW
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
//    public void bulkDataHub1m() {    // CAN ONLY BE USED WHEN DRIVE MOTORS HAVE ENCODERS CONNECTED!
//        // Read all the HUB1 status information in a single transaction
//        h1bulkData = expansionHub1.getBulkInputData();
//        // Parse front right mecanum motor data
//        frontRightMotorPos =  h1bulkData.getMotorCurrentPosition(HUB_MOTOR0);
//        frontRightMotorVel =  h1bulkData.getMotorVelocity(HUB_MOTOR0);
//        // Parse back right mecanum motor data
//        backRightMotorPos  =  h1bulkData.getMotorCurrentPosition(HUB_MOTOR1);
//        backRightMotorVel  =  h1bulkData.getMotorVelocity(HUB_MOTOR1);
//        // Parse front left mecanum motor data
//        frontLeftMotorPos  = -h1bulkData.getMotorCurrentPosition(HUB_MOTOR2);
//        frontLeftMotorVel  = -h1bulkData.getMotorVelocity(HUB_MOTOR2);
//        // Parse back left mecanum motor data
//        backLeftMotorPos   = -h1bulkData.getMotorCurrentPosition(HUB_MOTOR3);
//        backLeftMotorVel   = -h1bulkData.getMotorVelocity(HUB_MOTOR3);
//    } // bulkDataHub1m

    /*--------------------------------------------------------------------------------------------*/
    public void bulkDataHub1() {
        // Read all the HUB1 status information in a single transaction
        //h1bulkData = expansionHub1.getBulkInputData();
        // Parse right odometry encoder (front right motor port)
        rightOdometerPrev  = rightOdometerCount;
        rightOdometerCount = -h1bulkData.getMotorCurrentPosition(HUB_MOTOR0); // Must be POSITIVE when bot moves FORWARD
        // Parse left odometry encoder (front left motor port)
        leftOdometerPrev   = leftOdometerCount;
        leftOdometerCount  = h1bulkData.getMotorCurrentPosition(HUB_MOTOR2);  // Must be POSITIVE when bot moves FORWARD
        // Parse rear odometry encoder (back left motor port)
        rearOdometerPrev   = rearOdometerCount;
        rearOdometerCount  = h1bulkData.getMotorCurrentPosition(HUB_MOTOR3);  // Must be POSITIVE when bot moves RIGHT
    } // bulkDataHub1

    /*--------------------------------------------------------------------------------------------*/
//    public void bulkDataHub2() {
//        // Read all the HUB2 status information in a single transaction
//        h2bulkData = expansionHub2.getBulkInputData();
//        // Parse cascade-slide motor data
//        cascadeMotorPos = h2bulkData.getMotorCurrentPosition(HUB_MOTOR1);
//        cascadeMotorVel = h2bulkData.getMotorVelocity(HUB_MOTOR1);
//        // Parse scissor-lift motor data
//        scissorMotorPosR = -h2bulkData.getMotorCurrentPosition(HUB_MOTOR2);
//        scissorMotorVelR = -h2bulkData.getMotorVelocity(HUB_MOTOR2);
//        scissorMotorPosL = -h2bulkData.getMotorCurrentPosition(HUB_MOTOR3);
//        scissorMotorVelL = -h2bulkData.getMotorVelocity(HUB_MOTOR3);
//        // Parse magnetic limit switches for scissor-lift motors (limit reached = blue light = digital 0/false)
//        // NOTE: with motor ZeroPowerBehavior configured for BRAKE we're unlikely to coast thru the detection
//        // zone so don't currently implement any "latch" logic.
//        scissorMotorLimitL_on = !h2bulkData.getDigitalInputState(HUB_DIGPIN0);  // both 0&1 report LEFT status
//        scissorMotorLimitR_on = !h2bulkData.getDigitalInputState(HUB_DIGPIN2);  // both 2&3 report RIGHT status
//    } // bulkDataHub2

    /*--------------------------------------------------------------------------------------------*/
//    public double scissorMotorPower( double operatorPowerInput, int scissorMotorPos  )
//    {
//        double motorPower;
//        // Determine motor power while RAISING:
//        if( operatorPowerInput > 0.0 ) {
//            if( scissorMotorPos >= 17850 )
//                motorPower = 0.00;  // cannot lift any farther
//            else if( scissorMotorPos >= 17750 )
//                motorPower = 0.15;  // slow down for final few encoder counts
//            else
//                motorPower = operatorPowerInput;
//        }
//        // Determine motor power while LOWERING
//        else {
//            if( scissorMotorPos < scissorMotorPosMin )
//                motorPower = 0.00;  // cannot lower any farther
//            else if( scissorMotorPos < (scissorMotorPosMin + 150) )
//                motorPower = -0.10;  // slow down for final few encoder counts
//            else
//                motorPower = operatorPowerInput;
//        }
//        return motorPower;
//    } // scissorMotorPower

    /*--------------------------------------------------------------------------------------------*/
//    public double cascadeMotorPower( double operatorPowerInput )
//    {
//        double motorPower;
//        // Determine motor power while EXTENDING:
//        if( operatorPowerInput > 0.0 ) {
//            if( cascadeMotorPos >= 1200 )
//                motorPower = 0.00;  // cannot extend any farther
//            else if( cascadeMotorPos >= 1175 )
//                motorPower = 0.10;  // slow down for final few encoder counts
//            else
//                motorPower = operatorPowerInput;
//        }
//        // Determine motor power while RETRACTING
//        else {
//            if( cascadeMotorPos < 1 )
//                motorPower = 0.00;  // cannot retract any farther
//            else if( cascadeMotorPos < 50 )
//                motorPower = -0.15;  // slow down for final few encoder counts
//            else
//                motorPower = operatorPowerInput;
//        }
//        return motorPower;
//    } // cascadeMotorPower

    /*--------------------------------------------------------------------------------------------*/
    public void mecanumMotors( double frontLeft, double frontRight, double backLeft, double backRight )
    {
        double maxMotorDelta   = 0.25;  // from one cycle to the next, can't increase/decrease by more than this
        double frontLeftDelta  = frontLeft  - frontLeftLast;
        double frontRightDelta = frontRight - frontRightLast;
        double backLeftDelta   = backLeft   - backLeftLast;
        double backRightDelta  = backRight  - backRightLast;

        // Ensure we don't exceed the maximum allowed change for one cycle (incrase or decrease)
        if( Math.abs(frontLeftDelta) > maxMotorDelta)
            frontLeft  = frontLeftLast  + ((frontLeftDelta>0.0)?  maxMotorDelta : -maxMotorDelta);
        if( Math.abs(frontRightDelta) > maxMotorDelta)
            frontRight = frontRightLast + ((frontRightDelta>0.0)? maxMotorDelta : -maxMotorDelta);
        if( Math.abs(backLeftDelta) > maxMotorDelta)
            backLeft   = backLeftLast  + ((backLeftDelta>0.0)?  maxMotorDelta : -maxMotorDelta);
        if( Math.abs(backRightDelta) > maxMotorDelta)
            backRight  = backRightLast + ((backRightDelta>0.0)? maxMotorDelta : -maxMotorDelta);

        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        backLeftMotor.setPower( backLeft );
        backRightMotor.setPower( backRight );

        // Store these as the starting point for next cycle
        frontLeftLast  = frontLeft;
        frontRightLast = frontRight;
        backLeftLast   = backLeft;
        backRightLast  = backRight;

    } // mecanumMotors

    /*--------------------------------------------------------------------------------------------*/
    public void mecanumMotors2( double frontLeft, double frontRight, double backLeft, double backRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        backLeftMotor.setPower( backLeft );
        backRightMotor.setPower( backRight );
    } // mecanumMotors2

    /*--------------------------------------------------------------------------------------------*/
    public void mecanumMotors0()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        backLeftMotor.setPower( 0.0 );
        backRightMotor.setPower( 0.0 );
    } // mecanumMotors0

/*
    public void initGroundEffects()
    {
        // First argument is the number of LEDs we intend to use in the strip.
        leds = new DotStarLED( 20, ledClock, ledData );
        ledThread = new Thread(leds);
        // Start the color change monitor thread
        ledThread.start();
    } // initGroundEffects

    public void stopGroundEffects()
    {
        leds.terminate();
    } // stopGroundEffects

    public void setGroundEffectsColor( int newColor )
    {
        if(groundEffectsColor != newColor) {
            // Stop any current update.
            leds.stopUpdate();

            int red = (newColor & 0x00FF0000) >> 16;
            int green = (newColor & 0x0000FF00) >> 8;
            int blue = (newColor & 0x000000FF);

            // Set the new color
            for (int i = 0; i < leds.pixels.length; i++) {
                // leds.pixels is the public accessible array of pixel objects. We can set red / green
                // / blue fields, and call methods like reset().
                leds.pixels[i].red = red;
                leds.pixels[i].green = green;
                leds.pixels[i].blue = blue;
            }

            //Update the pixel colors.
            leds.update();
            groundEffectsColor = newColor;
        }
    } // setGroundEffectsColor
*/

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */
} /* HardwareHub1Hub2 */
