import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoMovement {
    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
    final double COUNTS_PER_INCH = 307.699557;

    public void yMovement(int power, double distance, double allowableDistance){
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double countsToGo = distance*COUNTS_PER_INCH;
        double error = allowableDistance*COUNTS_PER_INCH;
        double averageEncoders = (robot.rearLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition())/2;

        while( (averageEncoders >= Math.abs(countsToGo - error)) || (robot.readBackLeftTo() > error) || (robot.readBackRightTo() > error)){
            averageEncoders = (robot.frontLeft.getCurrentPosition() + robot.rearRight.getCurrentPosition())/2;
            robot.readBackRightTo();
            robot.readBackLeftTo();

            robot.setFrontLeftMotorPower(power * -1);
            robot.setRearLeftMotorPower(power * -1);
            robot.setFrontRightMotorPower(power);
            robot.setRearRightMotorPower(power);
        }
        robot.setFrontLeftMotorPower(0);
        robot.setRearLeftMotorPower(0);
        robot.setFrontRightMotorPower(0);
        robot.setRearRightMotorPower(0);
    }
    public void xMovement(int power, double distance, double allowableDistance){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double countsToGo = distance*COUNTS_PER_INCH;
        double error = allowableDistance*COUNTS_PER_INCH;
        double horizontalEncoder = robot.frontLeft.getCurrentPosition();

        while( (horizontalEncoder >= Math.abs(countsToGo - error)) || (robot.readBackLeftTo() > error) || (robot.readBackRightTo() > error)){
            horizontalEncoder = robot.frontLeft.getCurrentPosition();
            robot.readBackRightTo();
            robot.readBackLeftTo();

            robot.setFrontLeftMotorPower(power);
            robot.setRearLeftMotorPower(power);
            robot.setFrontRightMotorPower(power);
            robot.setRearRightMotorPower(power);
        }
        robot.setFrontLeftMotorPower(0);
        robot.setRearLeftMotorPower(0);
        robot.setFrontRightMotorPower(0);
        robot.setRearRightMotorPower(0);
    }
    public int distanceSensors() {
        return 10;
    }
}
