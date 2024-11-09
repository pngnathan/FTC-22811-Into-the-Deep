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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode is based on the RobotAutoDriveByTime_Linear sample code.
 * I have adapted it to utilize the RobotHardware class.
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Strafe right for 1 second
 *   - Drive diagonally to left for 2 seconds
 *   - Drive Backward for 1 Second
 *   - Drive in an arc to the left for 4 seconds
 *   - Stop
 *
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot", preselectTeleOp = "MorrisPOVDrive")
//@Disabled
public class MorrisAutoDriveByTime extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    /* Declare OpMode members. */
    int legNumber = 0;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds
        driveStraightForTime(.6, 3);

        // Step 2:  Spin right for 1.3 seconds
        turnForTime(.5, 1.3);

        // Step 3: Strafe right for 1 second
        strafeForTime(.5, 1);

        // Step 4: Drive diagonally to left for 2 seconds
        driveDiagonalForTime(.3,.3,2);

        // Step 5:  Drive backward for 1 second
        driveStraightForTime(-5,1);

        // Step 6: Drive in an arc to the left for 4 seconds
        driveArcForTime(.5,-1,4);

        // Step 7:  Stop
        driveStraightForTime(0, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
    * @param turn   Turn speed (-1.0 to 1.0) +ve is right
    * @param time   Time in seconds to complete the action.
    * This function is used for just turning.
    */
    private void turnForTime(double turn, double time) {
        legNumber += 1;
        robot.mechanumDrive(0, 0, turn);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Drive in an arc for a given time.
     * @param forward   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param turn      Turn speed (-1.0 to 1.0) +ve is right
     * @param time      Time in seconds to complete the action.
     */
    private void driveArcForTime(double forward, double turn, double time) {
        legNumber += 1;
        robot.mechanumDrive(forward, 0, turn);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * @param forward   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param time      Time in seconds to complete the action.
     * This version of driveForTime is used for driving in an arc.
     */
    private void driveStraightForTime(double forward, double time) {
        legNumber += 1;
        robot.mechanumDrive(forward, 0, 0);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Drive diagonally while facing forward.
     * @param forward   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left strafing (-1.0 to 1.0) +ve is right
     * @param time      Time in seconds to complete the action.
     */
    private void driveDiagonalForTime(double forward, double strafe, double time) {
        legNumber += 1;
        robot.mechanumDrive(forward, strafe, 0);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Drive to the right (+) or left (-) for a given time.
     * @param strafe    Right/Left strafing (-1.0 to 1.0) +ve is right
     * @param time      Time in seconds to complete the action.
     */
    private void strafeForTime(double strafe, double time) {
        legNumber += 1;
        robot.mechanumDrive(0, strafe, 0);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}