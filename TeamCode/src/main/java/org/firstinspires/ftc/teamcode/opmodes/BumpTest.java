package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class BumpTest extends CommandOpMode{
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


        //*engineer controls
        if (gamepad2.dpadUpWasPressed()) {
            robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY() + 0.5, robot.follower.getPose().getHeading()));
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.follower.setPose(new Pose(robot.follower.getPose().getX() - 0.5, robot.follower.getPose().getY(), robot.follower.getPose().getHeading()));
        }
        if (gamepad2.dpadRightWasPressed()) {
            robot.follower.setPose(new Pose(robot.follower.getPose().getX() + 0.5, robot.follower.getPose().getY(), robot.follower.getPose().getHeading()));
        }
        if (gamepad2.dpadDownWasPressed()) {
            robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY() - 0.5, robot.follower.getPose().getHeading()));
        }
        if (gamepad2.leftBumperWasPressed()){
            robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading() - Math.toRadians(0.5)));
        }
        if (gamepad2.rightBumperWasPressed()){
            robot.follower.setPose(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY(), robot.follower.getPose().getHeading() + Math.toRadians(0.5)));
        }




        //*telemetry
        telemetry.addData("Pose: ", robot.follower.getPose());
        telemetry.addData("angle: ", Math.toDegrees(robot.follower.getPose().getHeading()));

        super.loop(); //runs CommandOpMode's loop
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }
}
