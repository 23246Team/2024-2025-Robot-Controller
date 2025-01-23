package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoFar", group="Test")
public class AutoFar extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.
        robot.init();
        waitForStart();
        telemetry.addData("Initial Heading", robot.getHeading());
        telemetry.update();

        // Autonomous drive
        robot.AutoStrafe(-70, 0);


        // Add telemetry to show final heading after drive
        telemetry.addData("Final Heading", robot.getHeading());
        telemetry.update();


    }
}
