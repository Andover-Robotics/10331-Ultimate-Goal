package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Actual Auto", group = "Linear Opmode")

public class SenseAndParkAuto extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal; // i know that these are for the deadwheels (are these called DCMotor so we can configre them onto the hub?)
    final double COUNTS_PER_INCH = 307.699557;
    Servo colorSenseServo;
    ColorSensor color_sensor;
    DcMotor intakeMotor, outtakeMotor;  // did we initialize this yet
    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rightFrontDrive = "rf", rightBackDrive = "rb", leftFrontDrive = "lf", leftBackDrive = "lb";
    String verticalLeftEncoder = rightBackDrive, verticalRightEncoder = leftFrontDrive, horizontalEncoder= rightFrontDrive;

   OdometryGlobalCoordinatePosition globalPositionUpdate;
    double robot_movement_x_component = 0;
    double robot_movement_y_component = 0;
    double pivotCorrection = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive, verticalLeftEncoder, verticalRightEncoder, horizontalEncoder);
        //method above used for only mecanum drive 
        colorSenseServo = hardwareMap.get(Servo.class, "colorSenseServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor" );
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

       // servo.setPosition(servo.getPosition() + 90);
        // get a reference to the color sensor.
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();


        while(opModeIsActive()){

            //  DISCLAIMER: THIS AUTO ATM IS ONLY WORKING WHEN WE ARE IN THE RED TEAM POSITIONED AT THE FAR RIGHT STARTING ZONE!!
            // NOTE: think about making an auto that has adjustment to

            // this is where our actual code should go using all of the methods that we have
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

            /*
            notes about robot power:
            yo code peeps
            when you're testing the robot, you're gonna need to put the motors at half the speed you usually would
            the miter gears we're planning on using eventually will be a 1:1 ratio but the bevel gears we're borrowing from Lightning are 2:1, so the wheels will be spinning twice as fast as whatever you set the motor to
             */
            // NOTE: we need to figure out how to move (using the pivot correction and x component and y component (see goToPosition()) line 193
            // start coding here
            /*  GETTING THE ROBOT READY TO SENSE - pull up to the stack
            Drive forward 24 inches (plus space from robot to first tile line)
            Move left to sensor

            *ideally: move diagonally but depends on how easy it is to move wobble goal

             */


            /*
            IMPLEMENT COLOR SENSOR AND ITS USE
            if(sense color) {
                rings=4
                Drive 84 (3.5) forward 12 right (.5)

                Drive back 48 (2) inches
            }
                else {
                    tilt down
                    if(sense color){
                        rings = 1
                        Drive 60 (2.5) forward 12 (.5) left
                Drive back 24 inches
                    }
                    else{
                        rings = 0
                        Drive 36 (1.5) forward 12 (.5) right
                }
             } */
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

                    goToPosition(-12*COUNTS_PER_INCH,60*COUNTS_PER_INCH,0.5,0, 1*COUNTS_PER_INCH);
                    setPowerAllMotors(robot_movement_x_component ,  robot_movement_y_component ,pivotCorrection);
                    //  DISCLAIMER : SUBJ. TO CHANGE BECAUSE WE DONT KNOW IF THE WG WILL FALL OUT OF THE LITTLE POSITION THING WHEN WE STRAFE
                    //under the assumption that the wheels just move with setPower
                    //double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {

                    goToPosition(0*COUNTS_PER_INCH,24*COUNTS_PER_INCH,0.5,0, 1*COUNTS_PER_INCH);
                    setPowerAllMotors(robot_movement_x_component ,  robot_movement_y_component ,pivotCorrection);
                }else{
                    // if it doesnt sense yellow make decision that it is 0 (maybe if this doesnt work make another step to check if the servo can tilt down more to see grey
                    rings =0;
                    //Drive 36 (1.5) forward 12 (.5) right
                    goToPosition(12*COUNTS_PER_INCH,36*COUNTS_PER_INCH,0.5,0, 1*COUNTS_PER_INCH);
                    setPowerAllMotors(robot_movement_x_component ,  robot_movement_y_component ,pivotCorrection);

                }
            }else{
                /*
                Drive 84 (3.5) forward 12 right (.5)
                Drive back 48 (2) inches
                 */
                goToPosition(12*COUNTS_PER_INCH,84*COUNTS_PER_INCH,0.5,0, 1*COUNTS_PER_INCH);

                setPowerAllMotors(robot_movement_x_component ,  robot_movement_y_component ,pivotCorrection);


                goToPosition(0*COUNTS_PER_INCH,48*COUNTS_PER_INCH,0.5,0, 1*COUNTS_PER_INCH);

                setPowerAllMotors(robot_movement_x_component ,  robot_movement_y_component ,pivotCorrection);

            }



            /*ROBOT MOVES TO WHITE LINE AFTER DROPING OFF WG*/
            /*
            if(sense color) {
                rings=4
                Drive 84 (3.5) forward 12 right (.5)
                Drive back 48 (2)
            }*/



            int ringsTemp = 4; // for testing purposess
            int tempColor = 0 ; // initializing only requires ONE equals sign
            //conditional statements : while / if and else
            /*if(tempColor == color_sensor.argb() ){
              /*
              Drive 84 (3.5) forward 12 right (.5)
              Drive back 48 (2)
               */
            }
            /*
            else {
                tilt down
                if(sense color){
                    rings = 1
                    Drive 60 (2.5) forward 12 (.5) left
            Drive back 24 inches (1)
                }
                */
           /* else{
                colorSenseServo.setPosition(colorSenseServo.getPosition() + 90);
                if(color_sensor.argb()== tempColor ){
                    ringsTemp = 1;
                    /*
                    Drive 60 (2.5) forward 12 (.5) left
                   Drive back 24 inches (1)

                  */
              /*  }


            }
            /*
                else{
                    rings = 0
                    Drive 36 (1.5) forward 12 (.5) right
               }
            }

             */


        /*}*/

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void setPowerAllMotors (double xComp, double yComp, double pivCor){ // this is using the variables
        left_front.setPower(xComp +  yComp  - pivCor);
        right_front.setPower(-xComp + yComp + pivCor);
        left_back.setPower(-xComp + yComp - pivCor);
        right_back.setPower(xComp + yComp + pivCor);
    }
    // NOT USING THIS ANYMORE BECAUSE NOT GOOD IN METHOD FORM
    public int senseRings(){ // sensing rings on ground
      // servo use based off the assumption that servo starts at angle 0 degrees
      int rings = 4;
      int yellowValTemp = 0; // this is temp bc we dont actually know the val- subst later for range of values

      if(color_sensor.argb() < yellowValTemp){
          // this is at the height of 2 rings and it doesnt sense them it rotates the servo
         //move the servo down 45 degrees
           colorSenseServo.setPosition(colorSenseServo.getPosition() + 90); // going off teh basis of 90 degrees that moves in close to right above the stack
         if(color_sensor.argb() == yellowValTemp){ // this senses the 1 ring
             rings = 1;
         }else{
             // if it doesnt sense yellow make decision that it is 0 (maybe if this doesnt work make another step to check if the servo can tilt down more to see grey
             rings =0;
         }
      }
      // look into the argb color sensor stuff - does it use hex code instead of 3 singular numbers
      /* using RGB to determine yellow - will test when we get to in person
      if(color_sensor.red() > 200 && color_sensor.blue() > 200 && color_sensor.green() < 40)
       */
      return rings;
    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        
        while(opModeIsActive() && distance > allowableDistanceError){
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation(); // this returns : ?? an angle
            // maybe write a method that uses these 3 values t
            //get robots roation from 3-wheel odometry
        }
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
