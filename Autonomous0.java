package org.firstinspires.ftc.teamcode.Odometry;

/* FTC Team 7572 - Version 2.0 (01/08/2020)
 */
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

/**
 * Autonomous
 */

@Autonomous(name="Odometry UnitTest", group="7592")
//@Disabled
public class Autonomous0 extends LinearOpMode {

    HardwareHub1Hub2 robot = new HardwareHub1Hub2();

    //Files to access the algorithm constants
    File wheelBaseSeparationFile  = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double robotEncoderWheelDistance            = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * robot.COUNTS_PER_INCH;
    double horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    double robotGlobalXCoordinatePosition       = 0;  // (0.0 * robot.COUNTS_PER_INCH)
    double robotGlobalYCoordinatePosition       = 0;
    double robotOrientationRadians              = 0;

    int    unitTestNumber = 1;

    double power90 = 0.90;
    double power80 = 0.80;
    double power70 = 0.70;
    double power60 = 0.60;
    double power50 = 0.50;
    double power40 = 0.40;
    double power30 = 0.30;
    double power20 = 0.20;
    double power10 = 0.10;
    double power00 = 0.00;

    double xy_tol_200 = 2.00;
    double xy_tol_150 = 1.50;
    double xy_tol_100 = 1.00;
    double xy_tol_075 = 0.75;
    double xy_tol_050 = 0.50;

    double ang_tol_400 = 4.0;
    double ang_tol_200 = 2.0;
    double ang_tol_130 = 1.3;
    double ang_tol_110 = 1.1;
    double ang_tol_100 = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap);

        telemetry.addData("UnitTest:", "%s", "Forward (Y)" );
        telemetry.addData("State", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for team color/number
        // - Team Color: blue X button (blue) or red B button (red)
        // - Team Number: left/right bumpers
        while (!isStarted()) {
            if( gamepad1.y ) {
                unitTestNumber = 1;
                telemetry.addData("UnitTest:", "%s", "Forward (Y)" );
                telemetry.addData("State", "Ready");
                telemetry.update();
            }
            else if( gamepad1.b ) {
                unitTestNumber = 2;
                telemetry.addData("UnitTest:", "%s", "Right (B)" );
                telemetry.addData("State", "Ready");
                telemetry.update();
            }
            else if( gamepad1.x ) {
                unitTestNumber = 3;
                telemetry.addData("UnitTest:", "%s", "Turn (X)" );
                telemetry.addData("State", "Ready");
                telemetry.update();
            }
            else if( gamepad1.a ) {
                unitTestNumber = 4;
                telemetry.addData("UnitTest:", "%s", "ALL 3 (A)" );
                telemetry.addData("State", "Ready");
                telemetry.update();
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        unitTestOdometryDrive();

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify odometry-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        // Drive forward 10"
        if( (unitTestNumber == 1) || (unitTestNumber == 4) ) {
            driveToPosition( 10.0, 0.0, 0.0, power60, power30, xy_tol_050, ang_tol_100 );
            robot.mecanumMotors0();
            sleep( 1000 );
        }

        // Drive Right 10"
        if( (unitTestNumber == 2) || (unitTestNumber == 4) ) {
            double x_pos = (unitTestNumber == 4)? 10.0 : 0.0;
            driveToPosition( x_pos, 10.0, 0.0, power70, power30, xy_tol_050, ang_tol_100 );
            robot.mecanumMotors0();
            sleep( 1000 );
        }

        // Turn 90deg  (ensure x/y movement power is LESS THAN the turning power!)
        if( (unitTestNumber == 3) || (unitTestNumber == 4) ) {
            double x_pos = (unitTestNumber == 4)? 10.0 : 0.0;
            double y_pos = (unitTestNumber == 4)? 10.0 : 0.0;
            driveToPosition( x_pos, y_pos, 90.0, power30, power90, xy_tol_150, ang_tol_100 );
            robot.mecanumMotors0();
        }
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Move robot to specified target position/orientation
     * @param x_target (inches)
     * @param y_target (inches)
     * @param drive_angle (degrees; 0deg is straight ahead)
     * @param move_power
     * @param turn_power
     * @param xy_tol (inches)
     * @param ang_tol (deg)
     * @return boolean true/false for DONE?
     */
    private void driveToPosition( double x_target, double y_target, double drive_angle,
                                  double move_power, double turn_power,
                                  double xy_tol, double ang_tol ) {
        // Loop until we reach the target (or autonomous program aborts)
        while( opModeIsActive() ) {
            // Bulk-query all of the odometry encoder values at once
            robot.bulkDataHub1();
            // Compute updated robot position/orientation
            globalCoordinatePositionUpdate();
            // Power drivetrain motors to ove to where we WANT to be
            if( moveToPosition( x_target, y_target, drive_angle, move_power, turn_power, xy_tol, ang_tol ) )
                break;
        } // opModeIsActive()
    } // driveToPosition

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Compute instantaneous motor powers needed to move toward the specified target position/orientation
     * @param x_target (inches)
     * @param y_target (inches)
     * @param drive_angle (degrees; 0deg is straight ahead)
     * @param move_power
     * @param turn_power
     * @param xy_tol (inches)
     * @param ang_tol (deg)
     * @return boolean true/false for DONE?
     */
    private boolean moveToPosition( double x_target, double y_target, double drive_angle,
                                    double move_power, double turn_power,
                                    double xy_tol, double ang_tol ) {
        double smallAngleSpeed = 0.25;
        // Query the current robot position/orientation
        double x_world = robotGlobalYCoordinatePosition / robot.COUNTS_PER_INCH;  // inches
        double y_world = robotGlobalXCoordinatePosition / robot.COUNTS_PER_INCH;  // inches
        double angle_world = robotOrientationRadians;                             // radians
        // Compute distance and angle-offset to the target point
        double distanceToPoint   = Math.sqrt( Math.pow((x_target - x_world),2.0) + Math.pow((y_target - y_world),2.0) );
        double angleToPoint      = Math.atan2( (y_target - y_world), (x_target - x_world) ); // radians`
        double deltaAngleToPoint = AngleWrap( angleToPoint - angle_world );                  // radians
        // Compute x & y components required to move toward point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;
        // Compute absolute-value x and y distances for scaling
        double relative_x_abs = Math.abs( relative_x_to_point );
        double relative_y_abs = Math.abs( relative_y_to_point );
        // Compute movement power, while preserving shape/ratios of the intended movement direction
        double movement_x_power = (relative_x_to_point / (relative_y_abs + relative_x_abs)) * move_power;
        double movement_y_power = (relative_y_to_point / (relative_y_abs + relative_x_abs)) * move_power;
        // Compute robot orientation-angle error
        double robot_radian_err = AngleWrap( Math.toRadians(drive_angle) - angle_world );  // radians
        // If within 10deg of target angle, use reduced turn_power
        double small_rad_error = Math.abs( robot_radian_err / Math.toRadians(10.0) );
        double adjusted_turn_power = (small_rad_error <= 1.0)? (small_rad_error * smallAngleSpeed) : turn_power;
        double rotation_power = (robot_radian_err > 0.0)? adjusted_turn_power : -adjusted_turn_power;
        // Translate X,Y,rotation power levels into mecanum wheel power values
        double frontRight = movement_x_power - movement_y_power - rotation_power;
        double frontLeft  = movement_x_power + movement_y_power + rotation_power;
        double backRight  = movement_x_power + movement_y_power - rotation_power;
        double backLeft   = movement_x_power - movement_y_power + rotation_power;
        boolean ODOMETRY_DEBUG = false;
        if( true ) {
            // NOTE: Team 7592 robot correction factors
            frontLeft  *= 1.085;   // #4 fastest in RUN WITHOUT ENCODERS (noticably slower!)
            frontRight *= 0.915;   // #1 fastest
            backLeft   *= 0.925;   // #3 fastest
            backRight  *= 0.922;   // #2 fastest
        }
        // Determine the maximum motor power
        double maxWheelPower = Math.max( Math.max( Math.abs(backLeft),  Math.abs(backRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if( ODOMETRY_DEBUG ) {
            telemetry.addData("distanceToPoint", "%.2f in", distanceToPoint);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("relative_x_to_point", "%.2f in", relative_x_to_point);
            telemetry.addData("relative_y_to_point", "%.2f in", relative_y_to_point);
            telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", movement_x_power);
            telemetry.addData("movement_y_power", "%.2f", movement_y_power);
            telemetry.addData("rotation_power", "%.2f", rotation_power);
            telemetry.update();
            sleep(3500);  // so we can read it
        }
        // Are we within tolerance of our target position/orientation?
        if( (distanceToPoint <= xy_tol) && (Math.abs(robot_radian_err) <= Math.toRadians(ang_tol))  )
            return true;
        // NOT DONE!  Ensure no wheel powers exceeds 1.0
        if( maxWheelPower > 1.0 )
        {
            backLeft   /= maxWheelPower;
            backRight  /= maxWheelPower;
            frontLeft  /= maxWheelPower;
            frontRight /= maxWheelPower;
        }
        // Update motor power settings:
        robot.mecanumMotors2( frontLeft, frontRight, backLeft, backRight );
        return false;
    } // moveToPosition

    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angle
     * @return
     */
    public double AngleWrap( double angle ){
        while( angle < -Math.PI ) {
            angle += 2.0*Math.PI;
        }
        while( angle > Math.PI ){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        double leftChange  = robot.leftOdometerCount  - robot.leftOdometerPrev;
        double rightChange = robot.rightOdometerCount - robot.rightOdometerPrev;
        //Calculate Angle
        double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = robotOrientationRadians + changeInRobotOrientation;
        //Get the components of the motion
        double rawHorizontalChange = robot.rearOdometerCount - robot.rearOdometerPrev;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
        double p = ((rightChange + leftChange) / 2.0);
        double n = horizontalChange;
        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
    } // globalCoordinatePositionUpdate


} /* Autonomous0 */
