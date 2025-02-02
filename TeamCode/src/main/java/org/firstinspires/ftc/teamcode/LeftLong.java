package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="LeftLong", group="Left")
public class LeftLong extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.
        robot.init();
        waitForStart();
        telemetry.addData("Initial Heading", robot.getHeading());
        telemetry.update();

        // Autonomous drive
        robot.AutoDrive(-1.5,180);
        robot.AutoStrafe(-20,180);
        robot.AutoDrive(-47.3,180);
        robot.AutoStrafe(-8.5,180);
        robot.AutoDrive(51.1,180);
        robot.AutoStrafe(-8.5,180);
        robot.AutoDrive(-19.5,180,0.3);
        robot.AutoStrafe(13,180);
        robot.AutoDrive(-24.3,180);
        robot.AutoStrafe(-10,180);
        robot.AutoDrive(47,180);
        robot.AutoDrive(-16,180);
        robot.AutoStrafe(90.0,180,1, 0.5);
        robot.AutoDrive(15,180);



        // Add telemetry to show final heading after drive
        telemetry.addData("Final Heading", robot.getHeading());
        telemetry.update();


    }
}
