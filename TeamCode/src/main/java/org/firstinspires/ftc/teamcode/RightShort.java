package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RightShort", group="Right")
public class RightShort extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.

        robot.init();
        waitForStart();
        robot.AutoDrive(-1.5,180);

        robot.AutoStrafe(23,180);






    }
}
