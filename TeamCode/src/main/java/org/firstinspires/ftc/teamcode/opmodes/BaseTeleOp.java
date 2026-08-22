package org.firstinspires.ftc.teamcode.opmodes;


import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 *This is our base TeleOp class.
 * Red and Blue TeleOps that extend this should be created and put on the driver station.
 */
public class BaseTeleOp extends CommandOpMode {
    protected Robot robot = new Robot();

    /**
     * this is set by the constructor in red/blue teleop classes
     */
    protected boolean isRed;

    protected Pose closeShootPose = new Pose(39, 53, Math.toRadians(270));
    protected Pose gatePose = new Pose(144 - 14.4, 58.2, Math.toRadians(180 - 144.9));
    protected Pose farOpposingShootPose = new Pose(83, 15, Math.toRadians(180 - 110.1));
    protected Pose farAlliedShootPose = new Pose(57, 15, Math.toRadians(180 - 121.1));
    protected Pose farOpposingIntakePose = new Pose(144 - 12.0, 9.25, Math.toRadians(0));
    protected Pose farAlliedIntakePose = new Pose(12.0, 9.25, Math.toRadians(180));
    public BaseTeleOp(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public void init() {
        robot.initialize(isRed, hardwareMap);
        if (PoseSaver.autoWasRun) {
            robot.follower.setStartingPose(PoseSaver.endPose);
        } else {
            robot.follower.setStartingPose(robot.hpz);
        }
        PoseSaver.autoWasRun = false;
        if (!isRed) {
            closeShootPose.mirror();
            gatePose.mirror();
            farOpposingShootPose.mirror();
            farAlliedShootPose.mirror();
            farOpposingIntakePose.mirror();
            farAlliedIntakePose.mirror();
        }

        reset();
    }

    @Override
    public void start() {
        super.start();
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        schedule(robot.startManualDrive);
        schedule(robot.handleGate);
        schedule(robot.handleIntake);
        robot.beamBreaks.reset();
    }

    @Override
    public void loop() {
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //*autoaim
        if (gamepad1.bWasPressed()){
            robot.autoAiming = !robot.autoAiming;
        }

        //*shooting
        if (gamepad1.xWasPressed() && (robot.shooter.getFlywheelVelocity() > 0.1)) {
            schedule(robot.shoot);
        }

        //*close vs. far
        if (gamepad1.aWasPressed()){
            schedule(robot.shooter.setClose(true));
        }
//        if (gamepad1.yWasPressed()){
//            schedule(robot.shooter.setClose(false));
//        }

//        //*slow mode (engineer)
//        if (gamepad2.aWasPressed()){
//            robot.slowDrive = !robot.slowDrive;
//        }
        //*aim correction (engineer for now)
//        if (gamepad2.bWasPressed()){
//            schedule(robot.correctHeading);
//        }
        if (gamepad2.xWasPressed()){
            robot.follower.setPose(robot.limelight.getMt1Pose());
        }

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
//        if (gamepad2.yWasPressed()){
//            robot.limelightAim = !robot.limelightAim;
//        }

        //*intake
        if (gamepad1.right_trigger > 0.1){
            robot.setIntakeState(Robot.IntakeState.IN);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.setIntakeState(Robot.IntakeState.OUT);
        } else {
            robot.setIntakeState(Robot.IntakeState.OFF);
        }


        //*telemetry
        telemetry.addLine(robot.shooter.closeMode ? "----CLOSE----" : "||||FAR||||");
        telemetry.addLine(robot.autoAiming ? "AUTOAIM ON" : "Autoaim off");
        telemetry.addLine(robot.limelightAim ? "LIMELIGHT AIM" : "Odo Aim");
        telemetry.addData("ball amount: ", robot.beamBreaks.getBallCount());
        telemetry.addData("Pose: ", robot.follower.getPose());
        telemetry.addData("Goal Pose: ", robot.goalPose);
        telemetry.addData("angle: ", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("angle error: ", (robot.getOdoGoalAngleErrorDeg(false)));
        telemetry.addData("distance to goal: ", robot.getDistToGoal());


        //lab todo add automated drive controls from old code

        super.loop(); //runs CommandOpMode's loop
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }
}
