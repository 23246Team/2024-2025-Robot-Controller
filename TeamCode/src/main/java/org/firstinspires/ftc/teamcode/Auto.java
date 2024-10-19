package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto", group="Robot")
public class Auto extends LinearOpMode {
    private Hardware robot = new Hardware(this);
    private ElapsedTime Runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        Runtime.reset();
        // Run until the end of the match (driver presses STOP)
        robot.driveRobot(0, 0,0.5);
        while (opModeIsActive() && (Runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", Runtime.seconds());
            telemetry.update();
        }
        robot.driveRobot(0,0,0);
    }
}
