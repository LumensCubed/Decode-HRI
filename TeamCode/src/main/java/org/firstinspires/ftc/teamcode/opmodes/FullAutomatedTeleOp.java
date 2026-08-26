package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.branch;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;
import static com.seattlesolvers.solverslib.util.MathUtils.normalizeAngle;

import static org.firstinspires.ftc.teamcode.Util.getAbsoluteAngleRad;
import static org.firstinspires.ftc.teamcode.Util.getDistBetweenPoints;
import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.LinkedHashMap;
import java.util.function.BooleanSupplier;

public class FullAutomatedTeleOp extends CommandOpMode {
    protected Robot robot = new Robot();

    protected boolean isRed;

    protected Pose gatePose = new Pose(144 - 14.4, 58.2, Math.toRadians(180 - 144.9));

    public FullAutomatedTeleOp(boolean isRed) {
        this.isRed = isRed;
    }

    Command findBalls = sequential(
            robot.startTeleOpDrive,
            infinite(() -> robot.follower.setTeleOpDrive(0, 0, 0.5))
    )
            .requiring(robot)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setPriority(0);

    Command centerOnClosestBall = lazy(() -> {
    return infinite(() ->
            robot.follower.setTeleOpDrive(0,0,robot.getAimingPIDFOutput(robot.huskyLens.getClosestBallDeg()))
            )
            .until(() -> robot.huskyLens.getClosestBallDeg() < 2)
            .requiring(robot)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setPriority(1)
            ; //check that this actually updates
    }
    );
    Command driveToClosestBall = lazy(() -> {
        return infinite(() -> robot.follower.setTeleOpDrive(1,0,0))
                .until(() -> robot.huskyLens.getClosestBallDeg() > 5)
                .requiring(robot)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .setConflictBehavior(ConflictBehavior.QUEUE)
                .setPriority(2)
                ;
    }
    );
    LinkedHashMap<BooleanSupplier, Command> intakeCases = new LinkedHashMap<>();

    Command intake = branch(intakeCases).until(() -> robot.beamBreaks.getBallCount() == 3);

    Command moveToClosestShootingZone = lazy(() -> {
        PathChain robotPoseToShootingZone = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        robot.follower.getPose(),
                        getClosestShootPose()
                ))
                .setLinearHeadingInterpolation(robot.follower.getHeading(), getAbsoluteAngleRad(getClosestShootPose(), robot.goalPose))
                .build();
        return follow(robot.follower, robotPoseToShootingZone);
    }
    );


    public Pose getClosestShootPose(){ //red side
        Pose currentPose = robot.follower.getPose();
        Pose closePose;
        boolean redSide = currentPose.getX() >= 72;
        if (currentPose.getY() <= -abs(currentPose.getX() - 72) + 65) {
            closePose = new Pose(72, 65);
        } else {
            closePose = currentPose;
            boolean good = poseInShootingZone(closePose);
            while (!poseInShootingZone(closePose)) {
                closePose = new Pose(closePose.getX() + ((redSide ? -1 : 1) * 1), closePose.getY() + 1);
            }
        }
        Pose farPose;
        if (currentPose.getY() >= abs(currentPose.getX() - 72) + 31) { //this is wrong
            farPose = new Pose(72, 31);
        } else {
            farPose = currentPose;
            while (!poseInShootingZone(farPose)) {
                farPose = new Pose(farPose.getX() + ((redSide ? -1 : 1) * 0.5), (farPose.getY() <= 14 ? farPose.getY() : (farPose.getY() - 0.5)));
            }
        }
        return (getDistBetweenPoints(currentPose, closePose) < getDistBetweenPoints(currentPose, farPose) ? closePose : farPose);
    }
    boolean poseInShootingZone(Pose pose){
        return (pose.getY() >= abs(pose.getX() - 72) + 65) || (pose.getY() <= -abs(pose.getX() - 72) + 31);
    }



    @Override
    public void init() {
        robot.initialize(isRed, hardwareMap);
        intakeCases.put(() -> !robot.huskyLens.canSeeBalls(), findBalls);
        intakeCases.put(() -> robot.huskyLens.getClosestBallDeg() > 5, centerOnClosestBall);
        intakeCases.put(() -> true, driveToClosestBall);
        if (PoseSaver.autoWasRun) {
            robot.follower.setStartingPose(PoseSaver.endPose);
        } else {
            robot.follower.setStartingPose(robot.hpz);
        }
        PoseSaver.autoWasRun = false;
        if (!isRed) {
            gatePose = gatePose.mirror();
        }
        reset();
    }

    @Override
    public void start() {
        super.start();
        robot.update(0,0,0);
        schedule(robot.handleGate);
//        schedule(intake);
        robot.beamBreaks.reset();
    }

    @Override
    public void loop() {
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (robot.beamBreaks.getBallCount() == 3){
            if (poseInShootingZone(robot.follower.getPose())){
                schedule(sequential(turnTo(robot.follower, getAbsoluteAngleRad(robot.follower.getPose(), robot.goalPose)), robot.fastShoot, robot.localizeWithSmoothedLlPose));
            } else {
                schedule(sequential(moveToClosestShootingZone, robot.fastShoot, robot.localizeWithSmoothedLlPose));
            }
        }
        super.loop();
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }
}
