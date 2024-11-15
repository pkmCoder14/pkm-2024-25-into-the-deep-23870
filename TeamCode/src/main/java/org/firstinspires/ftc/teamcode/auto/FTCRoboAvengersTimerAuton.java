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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="RoboAvengers Timer Auton", group="Robot")
//@Disabled
public class FTCRoboAvengersTimerAuton extends LinearOpMode
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

    // Declare contants
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_SCORE_SPECIMEN        = 70 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET     = 95 * ARM_TICKS_PER_DEGREE;
    final double LIFT_TICKS_PER_MM = 537.7 / 120.0;
    final double LIFT_SCORING_IN_HIGH_BASKET = 475 * LIFT_TICKS_PER_MM;
    static final double     FORWARD_SPEED = 0.35;
    static final double     TURN_SPEED    = 0.5;
    static final double     STRAFE_SPEED  = 0.5;
    final double CLAW_OPEN   = 0.0;
    final double CLAW_CLOSED  = 1.0;


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
        claw  = hardwareMap.get(Servo.class, "claw");
        clawHead = hardwareMap.get(Servo.class, "clawHead");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Step 1. Claw closed
        while (opModeIsActive() && (runtime.milliseconds() < 1000) )
        {
            claw.setPosition(CLAW_OPEN);
        }
        sleep(500);
        telemetry.addData("Step 1: Claw closed", claw.getPosition());    //
        telemetry.update();
        runtime.reset();

        // Step 2. Lift the arm for scoring
        double armPosition = (int)ARM_SCORE_HIGH_BASKET;
        armMotor.setPower(0.3);
        double liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        liftMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.milliseconds() < 2500) )
        {
            claw.setPosition(CLAW_OPEN);
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 2: Robot arm angle ready for the scoring basket: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        // Step 3:  Drive forward for 1 seconds
        leftFrontDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 3: Path to basket: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 1.0))
        {
            claw.setPosition(CLAW_CLOSED);
        }

        telemetry.addData("Step 4: Sample dropped: ", "Complete");
        telemetry.update();
        sleep(500);
        runtime.reset();

        //Step 5 Reverse back
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Step 5: Reverse back: ", "Complete");
        telemetry.update();
        sleep(100);
        runtime.reset();

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);


//        armPosition = (int)ARM_CLEAR_BARRIER;
//        while (opModeIsActive() && (runtime.seconds() < 2.0) )
//        {
//            armMotor.setTargetPosition((int) (armPosition));
//            ((DcMotorEx) armMotor).setVelocity(2100);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        runtime.reset();
//        armMotor.setPower(0);
//        telemetry.addData("Path to collapsed arm: ", "Complete");
//        telemetry.update();
//        sleep(500);
    }
}
