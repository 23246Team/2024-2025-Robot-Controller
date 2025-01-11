package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Robot")
public class Measurements extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode() {
        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Strafe Encoder", robot.getStrafeEncoder());
            telemetry.addData("Drive Encoder", robot.getDriveEncoder());
            telemetry.update();
        }
        }
}
