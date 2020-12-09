/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.*;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//commenting things
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tank Drive TeleOp", group = "Iterative Opmode")
//@Disabled
public class MecanumDriveTrain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    //NEW FLYWHEELS
    private DcMotor intakeFlywheel, outtakeFlywheel;
    // boolean to see if Flywheels are running
    private boolean runFlywheelsForward = false, runFlywheelsBackwards = false, stop=false, runServosForward=false, runServosBackward=false;
    //Using ARC-Core's Mecanum Drive class, we initialized a Mecanum Drive as seen below
    private MecanumDrive mecanumDrive; // this is the object that we will be using to control the mecanum drive

    // New servo to move foundation
    private Servo servo;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(Motor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(Motor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(Motor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(Motor.class, "rightBackDrive");

        intakeFlywheel = hardwareMap.get(DcMotor.class, "intakeFlywheel");
        //outtakeFlywheel = hardwareMap.get(DcMotor.class, "outtakeFlywheel");

       // servo = hardwareMap.get(Servo.class, "servo");

        // Initialize our Mecanum Drive using our motors as parameters
        // Set default power of mecanum drive to 1.0
        //iveTrain = MecanumDrive.fromCrossedMotors(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, this, 89, 1120);;
        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive); //figure out with michael how to properly use the mecanum drive stuff
        //mecanumDrive.setRunMode(Motor.RunMode.PositionControl);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setRunMode(Motor.RunMode.RawPower); // setting direction for REGULAR motors?
        // what is position control (ask michael!)
        leftBackDrive.setRunMode(Motor.RunMode.RawPower);
        rightFrontDrive.setRunMode(Motor.RunMode.RawPower);
        rightBackDrive.setRunMode(Motor.RunMode.RawPower);

        /*leftFrontDrive.setDirection(Motor.Direction.FORWARD); // setting direction for REGULAR motors?
        leftBackDrive.setDirection(Motor.Direction.FORWARD);
        rightFrontDrive.setDirection(Motor.Direction.REVERSE);
        rightBackDrive.setDirection(Motor.Direction.REVERSE);*/

        // Flywheels move backwards to move blocks inward
        intakeFlywheel.setDirection(DcMotor.Direction.REVERSE);
        outtakeFlywheel.setDirection(DcMotor.Direction.REVERSE);

        /*leftFrontFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        intakeFlywheel.setPower(0); //since these are Dcmotors they can use the fucntion
        outtakeFlywheel.setPower(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    int count = 0;

    @Override
    public void loop() {
        count++;
        telemetry.addLine(((Integer)count).toString());
        telemetry.addData("LF Flywheel Power: ",  intakeFlywheel.getPower());

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y; // controls up and down movement
        //based off the assumption that these are giving numerical values between -1,0,1
        double strafe = gamepad1.left_stick_x; // the movement left to right of robot
        double turn = gamepad1.right_stick_x; //same with above


        leftFrontDrive.set(drive + strafe + turn);
        leftBackDrive.set(drive - strafe + turn);
        rightFrontDrive.set(drive - strafe - turn);
        rightBackDrive.set(drive + strafe - turn);

        // Strafe -- parameters: x and y coordinate to determine direction of movement
        // strafe gives x-direction of movement, drive gives y
        /*
        if (strafe < 0) {
            telemetry.addLine("Trying to strafe left");
            //these -1 and 1 values based off the assumption -1 is backwards and 1 is forwards 
            //https://gm0.org/en/stable/docs/software/mecanum-drive.html has a good diagram for understanding direction of wheels in relation to movemenet of robot
            leftFrontDrive.set(-1);
            rightFrontDrive.set(1);
            leftBackDrive.set(1);
            rightBackDrive.set(-1);
            //driveTrain.setStrafe(strafe, drive);
        }

        else if (strafe > 0) {
            telemetry.addLine("Trying to strafe right");
            leftFrontDrive.set(1);
            rightFrontDrive.set(-1);
            leftBackDrive.set(-1);
            rightBackDrive.set(1);
        }

        else {
            leftFrontDrive.set(0);
            rightFrontDrive.set(0);
            leftBackDrive.set(0);
            rightBackDrive.set(0);
            mecanumDrive.setMovementPower(drive);
        }*/


        // If turn is positive (joystick is pushed to the right), then rotate cw
        // If turn is negative (joystick is pushed to the left), then rotate ccw
        // NOTE: ADJUST FOR SENSITIVITY -- coordinate w/drive team
        /*
        if(turn > 0) {
            telemetry.addLine("Trying to turn");
            //driveTrain.rotateClockwise(10,1);
            mecanumDrive.driveRobotCentric(strafe,drive,turn);

            mecanumDrive.setRotationPower(1);
        }else if(turn < 0) {
            telemetry.addLine("Trying to turn");
            //driveTrain.rotateCounterClockwise(10,1);
            mecanumDrive.setRotationPower(-1);
        }else{
            mecanumDrive.setRotationPower(0);
        }
         */

        //when 'a' button is pressed and front flywheels are not running, set bool to true
        if (gamepad2.a) {
            runFlywheelsBackwards = false;
            runFlywheelsForward = true;
            // when 'a' button is pressed and front flywheels are running, set bool to false
        } //else if (gamepad2.a && runFlywheelsForward) {
        //runFlywheelsForward = false;
        //}

        else if (gamepad2.b) {
            runFlywheelsForward = false;
            runFlywheelsBackwards = true;
        }

        else if (gamepad2.x) {
            runFlywheelsForward = false;
            runFlywheelsBackwards = false;
        }

        //run flywheels at full power when bool true

        /*if (!runFlywheelsBackwards) {
            if (runFlywheelsForward) {
                leftFrontFlywheel.setPower(1);
                rightFrontFlywheel.setPower(1);
            } else {
                leftFrontFlywheel.setPower(0);
                rightFrontFlywheel.setPower(0);
            }
        }*/



        if (!runFlywheelsBackwards && !runFlywheelsForward) {
            intakeFlywheel.setPower(0);
            outtakeFlywheel.setPower(0);
        }else if (runFlywheelsForward) {
            intakeFlywheel.setPower(1);
            outtakeFlywheel.setPower(-1);
        }else {
            intakeFlywheel.setPower(-1);
            outtakeFlywheel.setPower(1);
        }

        //when 'b' button is pressed and back flywheels are not running, set bool to true
        /*if (gamepad2.b && !runFlywheelsBackwards && !runFlywheelsForward) {
            runFlywheelsBackwards = true;
            // when 'b' button is pressed and back flywheels are running, set bool to false
        } else if (gamepad2.b && runFlywheelsBackwards) {
            runFlywheelsBackwards = false;
        }*/

        //run flywheels at full power when bool true

        /*if (!runFlywheelsForward) {
            if (runFlywheelsBackwards) {
                leftFrontFlywheel.setPower(-1);
                rightFrontFlywheel.setPower(-1);
            } else {
                leftFrontFlywheel.setPower(0);
                rightFrontFlywheel.setPower(0);
            }
        }*/

        if (gamepad1.a) {
            runServosForward = true;
            runServosBackward = false;
        }else if (gamepad1.b) {
            runServosBackward = true;
            runServosForward = false;
        }else if (gamepad1.x) {
            runServosForward = false;
            runServosBackward = false;
        }

        if (!runServosBackward && !runServosForward) {
            servo.setPosition(servo.getPosition() + 90);
        }else if (runServosForward) {
            servo.setPosition(servo.getPosition() - 90);
        }else {
            servo.setPosition(servo.getPosition());
        }


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        /*
        leftBackDrive.setPower(leftbackPower);
        rightBackDrive.setPower(rightbackPower);
        leftFrontDrive.setPower(leftfrontPower);
        rightFrontDrive.setPower(rightfrontPower);
        */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
    //test

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    //test commit

}

