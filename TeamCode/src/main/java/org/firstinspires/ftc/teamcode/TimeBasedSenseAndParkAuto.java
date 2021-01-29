package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import  com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TimeBasedSenseAndParkAuto", group = "Linear Opmode")
public class TimeBasedSenseAndParkAuto extends LinearOpMode {

    private static MotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    MecanumDrive mecanumDrive;
    Servo colorSenseServo;
    ColorSensor color_sensor;
    private static final double mecanumCircumference = 32, flywheelCircumference = 16;
    private static final int ticksPerMecanum = 1120, ticksPerFlywheel = 538;
    //public enum SkyStoneStatus {NO_STONE, STONE, SKYSTONE}
    ElapsedTime runtime = new ElapsedTime();

    private final static double TILE_LENGTH = (24), FIELD_LENGTH = 6 * TILE_LENGTH;

    @Override
    public void runOpMode() {

        runtime.reset();

        telemetry.addLine("This started");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSenseServo = hardwareMap.get(Servo.class, "colorSenseServo");
        leftFrontDrive = new MotorEx(hardwareMap, "leftFrontDrive", Motor.GoBILDA.RPM_312);
        leftBackDrive = new MotorEx(hardwareMap, "leftBackDrive", Motor.GoBILDA.RPM_312);
        rightFrontDrive = new MotorEx(hardwareMap, "rightFrontDrive", Motor.GoBILDA.RPM_312);
        rightBackDrive = new MotorEx(hardwareMap, "rightBackDrive", Motor.GoBILDA.RPM_312);
        /*
        leftFrontFlywheel = hardwareMap.get(DcMotor.class, "leftFrontFlywheel");
        leftBackFlywheel = hardwareMap.get(DcMotor.class, "leftBackFlywheel");
        rightFrontFlywheel = hardwareMap.get(DcMotor.class, "rightFrontFlywheel");
        rightBackFlywheel = hardwareMap.get(DcMotor.class, "rightBackFlywheel");
         */

        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        waitForStart();

        while (opModeIsActive()) {
            leftFrontDrive.setRunMode(MotorEx.RunMode.PositionControl);
            rightFrontDrive.setRunMode(MotorEx.RunMode.PositionControl);
            leftBackDrive.setRunMode(MotorEx.RunMode.PositionControl);
            rightBackDrive.setRunMode(MotorEx.RunMode.PositionControl);

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition());
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());

            move(60,12,0);


            int rings = 4;
            int yellowValTemp = 0; // this is temp bc we dont actually know the val- subst later for range of values

            if(color_sensor.argb() < yellowValTemp){ // wasnt the right color
                // this is at the height of 2 rings and it doesnt sense them it rotates the servo
                //move the servo down 45 degrees
                colorSenseServo.setPosition(colorSenseServo.getPosition() + 90); // going off teh basis of 90 degrees that moves in close to right above the stack
                if(color_sensor.argb() == yellowValTemp){ // this senses the 1 ring
                    rings = 1;

                    //Drive 60 (2.5) forward 12 (.5) left
                    // robotPower (?) -> gear to wheel ratio
                    // under the assumption that 0 = forwards (for robot orientation)
                    // allowable distance error (put at 1 inch for now since we dont know what to do lol)

                    move(60,-12,0);
                    //  DISCLAIMER : SUBJ. TO CHANGE BECAUSE WE DONT KNOW IF THE WG WILL FALL OUT OF THE LITTLE POSITION THING WHEN WE STRAFE
                    //under the assumption that the wheels just move with setPower
                    //double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {

                    move(-24, 0, 0);
                }else{
                    // if it doesnt sense yellow make decision that it is 0 (maybe if this doesnt work make another step to check if the servo can tilt down more to see grey
                    //Drive 36 (1.5) forward 12 (.5) right
                    //already on line, don't drive back
                    move(36,12,0);

                }
            }else{
                /*
                Drive 84 (3.5) forward 12 right (.5)
                Drive back 48 (2) inches
                 */
                move(84,12,0);

                move(-48,0,0);

            }



        }

    }



    public static double findTotalTicks(int ticksPerRev, double circumference, double intendedDist) {


        double amtOfWheelUsed = intendedDist / circumference;

        double degrees = amtOfWheelUsed * 360;

        double finalTicks = degrees * ticksPerRev / 360;

        /*

        225/360 = x/1120
        x = 700

         */

        return finalTicks;
    }

    private static double inchesToCm(double inches) {
        return inches*2.54;
    }
    /*
    public static void flywheelIntake() {
        leftFrontFlywheel.setTargetPosition(leftFrontFlywheel.getCurrentPosition()+(int)findTotalTicks(ticksPerFlywheel,flywheelCircumference, 2*flywheelCircumference));
        rightFrontFlywheel.setTargetPosition(rightFrontFlywheel.getCurrentPosition()+(int)findTotalTicks(ticksPerFlywheel,flywheelCircumference, 2*flywheelCircumference));
    }
    public static void flywheelOuttake() {
        leftBackFlywheel.setTargetPosition(leftBackFlywheel.getCurrentPosition()+(int)findTotalTicks(ticksPerFlywheel,flywheelCircumference, 2*flywheelCircumference));
        rightBackFlywheel.setTargetPosition(rightBackFlywheel.getCurrentPosition()+(int)findTotalTicks(ticksPerFlywheel,flywheelCircumference, 2*flywheelCircumference));
    }
     */
    /*
    public static void strafeLeft(double distance) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
    }
    public static void strafeRight(double distance) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(distance)));
    }
    */
    public static void move(double drive, double strafe, double turn) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(drive) + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(strafe) + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(turn)))));
        leftBackDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(drive) - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(strafe) + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(turn)))));
        rightFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(drive) - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(strafe) - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(turn)))));
        rightBackDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(drive) + (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(strafe) - (int) findTotalTicks(ticksPerMecanum, mecanumCircumference, inchesToCm(turn)))));
        while (!leftBackDrive.atTargetPosition()||!leftFrontDrive.atTargetPosition()||!rightBackDrive.atTargetPosition()||!rightFrontDrive.atTargetPosition()) {
            leftFrontDrive.set(0.75);
            leftBackDrive.set(0.75);
            rightFrontDrive.set(0.75);
            rightBackDrive.set(0.75);
        }
        leftFrontDrive.stopMotor();
        leftBackDrive.stopMotor();
        rightFrontDrive.stopMotor();
        rightBackDrive.stopMotor();
    }
}