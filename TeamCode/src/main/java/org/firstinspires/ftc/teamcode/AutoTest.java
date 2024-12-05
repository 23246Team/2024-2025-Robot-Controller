package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoTest", group="Test")
public class AutoTest extends LinearOpMode {
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        // Initialize all the hardware, using the hardware class.
        robot.init();
        waitForStart();
        robot.AutoDrive(50, 0);


    }
}
