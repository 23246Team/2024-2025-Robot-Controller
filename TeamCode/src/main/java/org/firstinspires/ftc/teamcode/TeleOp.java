package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Robot")
public class TeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    Hardware robot = new Hardware(this);

    @Override
    public void runOpMode() {
        double arm = 0;
        // Initialize all the hardware, using the hardware class.
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update drive, turn, and strafe values from gamepad
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x*0.75;

            if (gamepad2.right_stick_y < 0){
                arm = gamepad2.right_stick_y*.90;
            }
            else if(gamepad2.right_stick_y > 0){
                arm = gamepad2.right_stick_y*.75;
            }
            else{
                arm = 0;
            }
            if (gamepad2.right_trigger!= 0){
                robot.setSliderPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger != 0){
                robot.setSliderPower(-gamepad2.left_trigger);
            } else {
                robot.setSliderPower(0);
            }
            if (gamepad2.right_bumper){
                robot.setIntakePower(-1);
            }
            else if (gamepad2.left_bumper){
                robot.setIntakePower(1);
            }
            else {
                robot.setIntakePower(0);
            }
            if(gamepad1.x){
                turn /= 0.75;

            }
            if(gamepad1.y){
                drive *= 0.15;
            }
            // Drive the robot using the updated values
            robot.driveRobot(drive, turn, strafe);
            robot.setArmPower(arm);

            // Update telemetry data
            updateTelemetry(drive, turn, strafe);
        }
    }

    /**
     * Updates the telemetry data with the current drive, turn, and strafe values.
     *
     * @param drive  The current drive value.
     * @param turn   The current turn value.
     * @param strafe The current strafe value.
     */
    private void updateTelemetry(double drive, double turn, double strafe) {
        telemetry.addData("Drive", "Left Stick");
        telemetry.addData("Turn", "Right Stick");
        telemetry.addData("-", "-------");
        telemetry.addData("Drive Power", "%.2f", drive);
        telemetry.addData("Turn Power", "%.2f", turn);
        telemetry.addData("Strafe Power", "%.2f", strafe);
        telemetry.update();
    }
}