package org.firstinspires.ftc.teamcode.opmodes.testsAndTuners;

import static org.firstinspires.ftc.teamcode.Util.getDistBetweenPoints;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * This finds the distance that the robot travels when going from max velocity to zero velocity
 * while braking at maximum strength. This is for bump detection.
 */
@TeleOp
public class DecelerationDistanceTuner extends CommandOpMode {
    private Robot robot = new Robot();
    Pose maxVelPose, stopPose;
    boolean atMax = false, done = false;

    @Override
    public void init() {
        telemetry.addLine("Make sure the robot has a lot of space");
        robot.initialize(false, hardwareMap);
        super.init();
        robot.follower.startTeleOpDrive();
    }

    @Override
    public void start() {
        super.start();
        robot.follower.setTeleOpDrive(1,0,0);
    }

    @Override
    public void loop() {
        telemetry.addData("velocity magnitude", robot.follower.getVelocity().getMagnitude());
        robot.follower.update();
        if (robot.follower.getVelocity().getMagnitude() > robot.MAX_VELOCITY && !atMax){
            robot.follower.setTeleOpDrive(-1,0,0);
            maxVelPose = robot.follower.getPose();
            atMax = true;
        }
        if (robot.follower.getVelocity().getMagnitude() < 5 && atMax && !done){
            robot.follower.setTeleOpDrive(0,0,0);
            stopPose = robot.follower.getPose();
            done = true;
        }
        if (done){
            telemetry.addData("distance between max velocity and stop", getDistBetweenPoints(maxVelPose, stopPose));
            telemetry.addData("max velocity pose", maxVelPose);
            telemetry.addData("stop pose", stopPose);

        }
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
