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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Controller Map", group="Linear Opmode")
@Disabled
public class LinearOpModeBase extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple leftFront = null; //DcMotorSimple because it is connected to SPARK Mini
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotorEx shooterMotor = null;   //DcMotorEx offers extended capabilities such as setVelocity()
    private Servo intakeServo = null;
    private Servo shooterFlipper = null;
    //private Servo intakeLatch = null;     //Intake Latch is hopefully not needed.
    private Servo wobbleClaw = null;
    private Servo wobbleLift = null;
    private Servo conveyor  = null;


    ToggleMdP toggleClaw = new ToggleMdP();
    ToggleMdP toggleLift = new ToggleMdP();

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotorSimple.class, "lf_drive");
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        leftBack  = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        shooterFlipper = hardwareMap.get(Servo.class,"shooterFlipper_servo");
        //intakeLatch = hardwareMap.get(Servo.class,"intakeLatch_servo");
        wobbleClaw = hardwareMap.get(Servo.class, "wobble_claw");
        wobbleLift = hardwareMap.get(Servo.class, "wobble_lift");
        conveyor = hardwareMap.get(Servo.class, "conveyor_servo");


        //Set Motor  and servo Directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intakeServo.setDirection(Servo.Direction.FORWARD);
        shooterFlipper.setDirection(Servo.Direction.FORWARD);
        //intakeLatch.setDirection(Servo.Direction.FORWARD);
        wobbleClaw.setDirection(Servo.Direction.FORWARD);
        wobbleLift.setDirection(Servo.Direction.FORWARD);
        conveyor.setDirection(Servo.Direction.FORWARD);


        //Set brake or coast modes. Drive motors should match SPARK Mini switch
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        return;
    }

    //controller map

    public boolean visionButton(){
        return gamepad1.a;
    }

    public double rotationVal(){
        double scalingFactor = 0.4;
        return gamepad1.right_stick_x*scalingFactor;
    }

    public void holonomicDriveAuto(double robotSpeed, double movementAngle, double rotationSpeed){

        double leftFrontSpeed = robotSpeed*Math.cos(movementAngle + (Math.PI/4)) + rotationSpeed;
        double rightFrontSpeed = robotSpeed*Math.sin(movementAngle + (Math.PI/4)) - rotationSpeed;
        double leftBackSpeed = robotSpeed*Math.sin(movementAngle + (Math.PI/4)) + rotationSpeed;
        double rightBackSpeed = robotSpeed*Math.cos(movementAngle + (Math.PI/4)) - rotationSpeed;

        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftBack.setPower(leftBackSpeed);
        rightBack.setPower(rightBackSpeed);
    }

  /*  public double robotMecanumSpeed(){
        double scalingFactor = 1;
        double speed;
        //put some code here for stick magnitude
        return speed;
    }
    */

    public double getAngle(){
        return 0; //replace with angle from gyro
    }

    public void driveTankAuto(double leftSpeed, double rightSpeed){
        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);

        rightFront.setPower(rightSpeed);
        rightBack.setPower(rightSpeed);
    }

    public void driveStraight(double targetDistance, double speed){
        double startAngle = getAngle();
        double currentAngle = startAngle;
        double currentDistance = 0;
        double leftSpeed = speed;
        double rightSpeed = speed;

        double hysteresis = 2; //minimum angle to correct for
        double correctionConstant = 5;

        while(currentDistance < targetDistance){
            currentAngle = getAngle();

            if(currentAngle > (startAngle + hysteresis)){  //if turning left, correct by turning right
                leftSpeed = speed + correctionConstant;
                rightSpeed = speed - correctionConstant;
            }
            else if(currentAngle < (startAngle - hysteresis)){  //if turning right, correct by turning left
                leftSpeed = speed - correctionConstant;
                rightSpeed = speed + correctionConstant;
            }
            else{
                leftSpeed = speed;
                rightSpeed = speed;
            }

            driveTankAuto(leftSpeed,rightSpeed);

            currentDistance = 0; //replace with value from encoder calculations
        }

        stopDriveMotors();

    }

    private void stopDriveMotors() {
        //set power to 0 for all drive motors
    }


}
