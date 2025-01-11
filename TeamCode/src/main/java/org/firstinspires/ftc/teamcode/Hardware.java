/* Copyright (c) 2022 FIRST. All rights reserved.
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
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class Hardware {

    private static final double TICKS_PER_INCH = 345;
    // Proportional control instances

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftBackDrive  = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor arm   = null;
    private CRServo intake  = null;
    private DcMotor leftSlider  = null;
    private DcMotor rightSlider  = null;
    public IMU imu;
    private DcMotor driveEncoder;
    private DcMotor strafeEncoder;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }




    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()      {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "front_left_motor");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "front_right_motor");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "back_left_motor");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "back_right_motor");
        intake = myOpMode.hardwareMap.get(CRServo.class, "intake");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        leftSlider = myOpMode.hardwareMap.get(DcMotor.class, "leftSlider");
        rightSlider = myOpMode.hardwareMap.get(DcMotor.class, "rightSlider");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "back_left_motor");
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "front_left_motor");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    private boolean opModeInInit() {
        return false;
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn, double Strafe) {
        // Combine drive and turn for blended motion.

        double leftBack = Drive + Turn + Strafe;
        double rightBack = Drive - Turn - Strafe;
        double leftFront = Drive - Turn + Strafe;
        double rightFront = Drive + Turn - Strafe;



        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(leftBack), Math.abs(rightBack));
        if (max > 1.0)
        {
            leftBack /= max;
            rightBack /= max;
            rightFront /= max;
            leftFront /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(leftBack,rightBack,rightFront,leftFront);
    }


    public void setDrivePower(double leftBackWheel, double rightBackWheel, double leftFrontWheel, double rightFrontWheel) {
        // Output the values to the motor drives.
        leftBackDrive.setPower(leftBackWheel);
        leftFrontDrive.setPower(leftFrontWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }
    public void setArmPower (double power) {
        arm.setPower(power);
    }

    public void setIntakePower (double power) {
        intake.setPower(power);
    }

    public void setSliderPower (double power) {
        leftSlider.setPower(-power);
        rightSlider.setPower(power);
    }
    public IMU getImu() {
        return imu;
    }

    public double getStrafeEncoder() {
        return (strafeEncoder.getCurrentPosition()/TICKS_PER_INCH);
    }

    public double getDriveEncoder() {
        return (driveEncoder.getCurrentPosition()/TICKS_PER_INCH);
    }
    public void AutoDrive(double drive, double heading) {
        double rampUp = Math.abs(drive) * 0.6;
        double speed = 0.3; // Initial speed
        double speedIncrement = 0.05; // Speed increment
        double direction = Math.signum(drive); // Determine the direction (1 for forward, -1 for backward)

        while (Math.abs(getDriveEncoder()) < math.abs(rampUp) && myOpMode.opModeIsActive()) {
            double error = getSteeringCorrection(heading, P_DRIVE_GAIN);
            speed = Math.min(0.5, speed + speedIncrement);
            driveRobot(speed * direction, error, 0);
        }

        while (Math.abs(getDriveEncoder()) < Math.abs(drive) && myOpMode.opModeIsActive()) {
            double error = getSteeringCorrection(heading, P_DRIVE_GAIN);
            double remainingDistance = Math.abs(drive) - Math.abs(getDriveEncoder());
            double decelerationSpeed = Range.clip(remainingDistance / Math.abs(drive) * 0.2, 0.2, 0.7); // Decrease speed as it approaches the target
            driveRobot(decelerationSpeed * direction, error, 0);
        }

        driveRobot(0, 0, 0); // Stop the robot after driving
}

    public void AutoStrafe(double strafe, double heading) {
        double rampUp = Math.abs(strafe) * 0.6;
        double speed = 0.3; // Initial speed
        double speedIncrement = 0.05; // Speed increment
        double direction = Math.signum(strafe); // Determine the direction (1 for right, -1 for left)

        while (Math.abs(getStrafeEncoder()) < Math.abs(rampUp) && myOpMode.opModeIsActive()) {
            double error = getSteeringCorrection(heading, P_DRIVE_GAIN);
            speed = Math.min(0.5, speed + speedIncrement);
            driveRobot(0, error, speed * direction);
        }

        while (Math.abs(getStrafeEncoder()) < Math.abs(strafe) && myOpMode.opModeIsActive()) {
            double error = getSteeringCorrection(heading, P_DRIVE_GAIN);
            double remainingDistance = Math.abs(strafe) - Math.abs(getStrafeEncoder());
            double decelerationSpeed = Range.clip(remainingDistance / Math.abs(strafe) * 0.2, 0.2, 0.7); // Decrease speed as it approaches the target
            driveRobot(0, error, decelerationSpeed * direction);
        }

        driveRobot(0, 0, 0); // Stop the robot after strafing
    }

    public void AutoTurn(double turn) {
        // Implement the turn logic here

    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        myOpMode.telemetry.addData("Heading: ", orientation.getYaw(AngleUnit.DEGREES));
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    static final double     P_DRIVE_GAIN           = 0.0003;     // Larger is more responsive, but also less stable
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        // Determine the heading current error
        double headingError = desiredHeading - getHeading(); // Positive error means the robot is too far to the right

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360; // Make error within +/- 180 degrees
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1); // Return the steering correction
    }
}
