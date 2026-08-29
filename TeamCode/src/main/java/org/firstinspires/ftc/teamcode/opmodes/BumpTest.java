

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Util.getDistBetweenPoints;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class BumpTest extends CommandOpMode {
    private Robot robot = new Robot();


    @Override
    public void init() {
        robot.initialize(false, hardwareMap);
        if (PoseSaver.autoWasRun) {
            robot.follower.setStartingPose(PoseSaver.endPose);
        } else {
            robot.follower.setStartingPose(robot.hpz);
        }
        PoseSaver.autoWasRun = false;
        reset();
    }

    @Override
    public void start() {
        super.start();
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        schedule(robot.handleGate);
        schedule(robot.handleIntake);
        robot.beamBreaks.reset();
    }

    @Override
    public void loop() {
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.aWasPressed()) robot.slowDrive = !robot.slowDrive;
        if (gamepad1.bWasPressed()) robot.follower.setPose(robot.hpz);

        //*telemetry
        telemetry.addData("Pose: ", robot.follower.getPose());
        telemetry.addData("velocity magnitude: ", robot.follower.getVelocity().getMagnitude());
        telemetry.addData("angular magnitude: ", robot.follower.getAngularVelocity());
        telemetry.addData("normal pos", robot.robotMax.getPosition());
        telemetry.addData("predicted pos", robot.robotMax.getPosition());
        telemetry.addData("inside line 1-2", robot.robotMax.isInside(robot.line12) || robot.predictedRobotMax.isInside(robot.line12));


        super.loop(); //runs CommandOpMode's loop
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }
}
