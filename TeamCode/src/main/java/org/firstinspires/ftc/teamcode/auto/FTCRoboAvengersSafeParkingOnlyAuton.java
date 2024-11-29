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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RoboAvengers Safe Parking Only Auton", group="Robot")
//@Disabled
public class FTCRoboAvengersSafeParkingOnlyAuton extends LinearOpMode
{
    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null; //the left back drivetrain motor
    public DcMotor  rightBackDrive   = null; //the right back drivetrain motor
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  liftMotor        = null;
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null; //the claw head servo//the slider

    // Declare constants
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_CLEAR_BARRIER         = 25 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET     = 100 * ARM_TICKS_PER_DEGREE;
    final double LIFT_TICKS_PER_MM = 537.7 / 120.0;
    final double LIFT_SCORING_IN_HIGH_BASKET = 475 * LIFT_TICKS_PER_MM;
    static final double     FORWARD_SPEED = 0.3;
    final double CLAW_CLOSED = 0.0;
    final double CLAW_OPEN = 1.0;
    //Calculate circumference of the wheel
    final double circumference = Math.PI * 104;
    final double WheelTurnsToBasket = 495.3/circumference; //Step 3
    final int EncoderCountToBasket = (int)(WheelTurnsToBasket * 537.7);

    //Reduced from 24.5 to 23
    final double WheelTurnsFromBasket = 584.2/circumference;
    final int EncoderCountFromBasket = (int)(WheelTurnsFromBasket * 537.7);

    final double WheelTurnsFromBasket2 = 2336.8/circumference;
    final int EncoderCountFromBasket2 = (int)(WheelTurnsFromBasket2 * 537.7);

    /* Variables that are used to set the arm to a specific position */
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.dcMotor.get("frontLeftMotor");
        leftBackDrive   = hardwareMap.dcMotor.get("backLeftMotor");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRightMotor");
        rightBackDrive  = hardwareMap.dcMotor.get("backRightMotor");
        armMotor        = hardwareMap.dcMotor.get("left_arm");
        liftMotor       = hardwareMap.dcMotor.get("liftMotor");
        claw            = hardwareMap.servo.get("claw");
        clawHead        = hardwareMap.servo.get("clawHead");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //getCurrent > 5 then reset encoder

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //Delete and try
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Wait for the game to start (driver presses START)
        waitForStart();
        int count = 0;

        while (opModeIsActive() && count < 1 ) // Do not change as we require time for arm to stabilize
        {
            // Step 1. Claw closed
            //runtime.reset();
            claw.setPosition(CLAW_CLOSED);
            telemetry.addData("Step 1: Claw closed", claw.getPosition());
            telemetry.update();
            sleep(500);

            // Step 2. Lift and extend the arm for scoring
            double armPosition = (int)ARM_SCORE_HIGH_BASKET;
            double liftPosition = LIFT_SCORING_IN_HIGH_BASKET;

            armMotor.setTargetPosition((int) (armPosition));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setPower(0.4);

            liftMotor.setTargetPosition((int) (liftPosition));
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setPower(0.4);

            //runtime.reset();
            while (armMotor.isBusy() || liftMotor.isBusy() )
            {
                telemetry.addData("Step 2: Robot arm ready for the top scoring basket: ", "Complete");
                telemetry.update();
            }
            //armMotor.setPower(0);
            //liftMotor.setPower(0);

            sleep(500); //[TBT] Reduced from 250 to 100

            // Step 3:  Drive forward towards the basket
            leftFrontDrive.setTargetPosition(EncoderCountToBasket);
            rightFrontDrive.setTargetPosition(EncoderCountToBasket);
            leftBackDrive.setTargetPosition(EncoderCountToBasket);
            rightBackDrive.setTargetPosition(EncoderCountToBasket);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);

            while(leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())
            {
                telemetry.addData("Step 3: Path to basket: ", "Complete");
                telemetry.update();
            }

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            sleep(500);
            //runtime.reset();

            // Step 4 Sample drop in top basket
            clawHead.setPosition(0.9);
            telemetry.addData("Step 4: Claw rotated", clawHead.getPosition());
            telemetry.update();
            sleep(500);
            runtime.reset();
            claw.setPosition(CLAW_OPEN); //[TBT] Moved outside the while loop
            telemetry.addData("Step 4: Sample dropped: ", "Complete");
            telemetry.update();
            sleep(500); //[TBT] Reduced from 500 to 100
            //runtime.reset();

            //Step 5 Reverse the Robot
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            leftFrontDrive.setTargetPosition(EncoderCountFromBasket2);
            rightFrontDrive.setTargetPosition(EncoderCountFromBasket2);
            leftBackDrive.setTargetPosition(EncoderCountFromBasket2);
            rightBackDrive.setTargetPosition(EncoderCountFromBasket2);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);

            while(leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())
            {
                telemetry.addData("Step 5: Reverse the robot: ", "Complete");
                telemetry.update();
            }

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            sleep(500);
            //runtime.reset();

            // Step 17 Reset claw and claw Head
            claw.setPosition(CLAW_CLOSED);
            telemetry.addData("Step 15: Second sample dropped: ", "Complete");
            telemetry.update();
            sleep(100);

            clawHead.setPosition(0.65);
            telemetry.addData("Step 15: Second sample dropped: ", "Complete");
            telemetry.update();
            sleep(250);


            // Step 17. Bring arm and lift to zero position
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            liftPosition = 0.0;

            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setTargetPosition((int) (liftPosition));
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setPower(0.5);

            armMotor.setTargetPosition((int) (armPosition));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setPower(0.5);

            while (armMotor.isBusy() || liftMotor.isBusy() )
            {
                telemetry.addData("Step 18: Robot arm closed: ", "Complete");
                telemetry.update();
            }

            sleep(100);//[TBT] Reduced from 250 to 100
            //runtime.reset();

            count++;
        }

        armMotor.setPower(0);
        liftMotor.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        /*send telemetry data to FTC Dashboard
        better to use target position and current position for motor position on FTC dashboard on mecanum drivetrain
        overlap the same motors to check if there is any slippage between what you set and what is actually happening
        better to use this if you are using timer or encoder based auton with your mecanum drivetrain*/
        telemetry.addData("Left Front Motor Target Position: ", leftFrontDrive.getTargetPosition());
        telemetry.addData("Right Front Motor Target Position: ", rightFrontDrive.getTargetPosition());
        telemetry.addData("Left Back Motor Target Position: ", leftBackDrive.getTargetPosition());
        telemetry.addData("Right Back Motor Target Position: ", rightBackDrive.getTargetPosition());

        telemetry.addData("Left Front Motor Position: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("Right Front Motor Position: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Left Back Motor Position: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Back Motor Position: ", rightBackDrive.getCurrentPosition());

        telemetry.addData("Left Front Motor Current:",((DcMotorEx) leftFrontDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Motor Current:",((DcMotorEx) rightFrontDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Back Motor Current:",((DcMotorEx) leftBackDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Back Motor Current:",((DcMotorEx) rightBackDrive).getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
        telemetry.addData("Arm Motor Current:",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Slide Motor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Slider Target Position",liftMotor.getTargetPosition());
        telemetry.addData("Slider Current position", liftMotor.getCurrentPosition());

        telemetry.addData("Claw Servo position:",claw.getPosition());
        telemetry.addData("Claw Head Servo position:",clawHead.getPosition());
    }
}
