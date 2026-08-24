package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.seattlesolvers.solverslib.util.MathUtils.normalizeAngle;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


import java.util.ArrayList;
import java.util.List;

/**
 * This holds all of our subsystem classes and puts together Commands using them.
 */
@Configurable
public class Robot {
    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public HuskyLens huskyLens = new HuskyLens();
    public Follower follower; //! change if needed
    public BeamBreaks beamBreaks = new BeamBreaks();
    public Kickstand kickstand = new Kickstand();
    public Limelight limelight = new Limelight();
    /**
     * if the robot should automatically aim to the goal.
     */
    public boolean autoAiming = false;
    public boolean limelightAim = false;
    public Pose goalPose;
    public Pose redGoal = new Pose(138, 138);
    public Pose hpz;
    public Pose redHpz = new Pose(10.5, 10.5, 0);
    double forwardInput, rightInput, rotateInput = 0;
    public boolean isShooting = false;
    public boolean slowDrive = false;
    public static double headingKP = 0.02;
    public static double headingKI = 0;
    public static double headingKD = 0.02;
    public static double headingKF = 0.03;
    public boolean isRed;
    private double savedOdoAngleDeg;
    ArrayList<Pose> limelightPoses = new ArrayList<>(10);
    int posesRead = 0;


    //*movement commands


    public Command aimAndStoreHeading() {
        return Command.build()
                .setStart(() -> {
                    autoAiming = true;
                    limelightAim = false;
                })
                .setDone(() -> abs(getOdoGoalAngleErrorDeg(false)) < 0.5) //doesn't account for overshoot
                .setEnd((endCondition) -> {
                    autoAiming = false;
                    savedOdoAngleDeg = Math.toDegrees(follower.getPose().getHeading());
                })
                ;
    }
    public Command correctHeadingWithLimelight() {
        return Command.build()
                .setStart(() -> {
                    autoAiming = true;
                    limelightAim = true;
                })
                .setDone(() -> abs(limelight.getTx()) < 0.5) //doesn't account for overshoot
                .setEnd((endCondition) -> {
                    autoAiming = false;
                    follower.setHeading(savedOdoAngleDeg);
                })
                ;
    }
    @Deprecated
    public Command correctHeading = sequential(
            aimAndStoreHeading(),
            waitMs(1000),
            correctHeadingWithLimelight()
    )
            .setPriority(2)
            .setConflictBehavior(ConflictBehavior.OVERRIDE)
            ;

    /**
     * runs drive based on input from the update method
     */
    public Command handleDriveInput = infinite(() -> {
        if (autoAiming) {
            if (limelightAim && limelight.canSeeGoal()) {
                follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput(limelight.getTx()));
            } else {
                follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput(getOdoGoalAngleErrorDeg(false)));
            }
        } else {
            follower.setTeleOpDrive(forwardInput, rightInput, rotateInput);
        }
    });

    public Command startTeleOpDrive = instant(() -> follower.startTeleOpDrive());

    public Command driveOff = instant(() -> follower.setTeleOpDrive(0,0,0));

    /**
     * starts TeleOp drive, then handles drive input
     */
    public Command startManualDrive = sequential(
            startTeleOpDrive,
            handleDriveInput
    )
            .requiring(follower)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            ;

    //*shooting commands
    Command setShooting(boolean shooting) {
        return instant(() -> {
            isShooting = shooting;
            if (!shooting){
                beamBreaks.reset();
            }
        });
    }

    Command setAiming(boolean aiming) {
        return instant(() -> autoAiming = aiming);
    }


    /**
     * immediately intakes, then closes gate
     */
    public Command fastShoot = sequential(
            setShooting(true),
            intake.setIn,
            waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false),
            setAiming(false)
            )
            .requiring(intake, shooter)
            .setPriority(2);


    /**
     * uses a moving average filter to take the average of 10 limelight mt1 poses
     * and then sets the current pose to it
     */
    public Command localizeWithSmoothedLlPose = lazy(() -> {
                return sequential(
                        driveOff,
                        waitUntil(() -> (follower.getVelocity().getMagnitude() < 0.1) && follower.getAngularVelocity() < 0.1),
                        Command.build()
                                .setStart(() -> {
                                    limelightPoses.clear();
                                    posesRead = 0;
                                })
                                .setExecute(() -> {
                                    limelightPoses.add(limelight.getMt1Pose());
                                    posesRead++;
                                })
                                .setDone(() -> posesRead == 10)
                                .setEnd(endCondition -> {
                                    double xSum = 0, ySum = 0, sinSum = 0, cosSum = 0;
                                    for (Pose pose : limelightPoses) {
                                        xSum += pose.getX();
                                        ySum += pose.getY();
                                        sinSum += Math.sin(pose.getHeading());
                                        cosSum += Math.cos(pose.getHeading());
                                    }
                                    follower.setPose(new Pose(
                                            xSum / limelightPoses.size(),
                                            ySum / limelightPoses.size(),
                                            Math.atan2(sinSum, cosSum)
                                    ));
                                }))
                        ;
            }
    )
            .requiring(follower)
            .setPriority(1)
            ;


    /**
     * waits for gate to open, shoots, then closes gate
     */
    public Command slowShoot = sequential(
            setShooting(true),
            intake.turnOff,
            shooter.open,
            waitMs(300), //robot todo change to waitUntil(gateIsOpen) once it's working
            intake.setIn,
            waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false),
            setAiming(false)
            )
            .requiring(intake, shooter)
            .setPriority(2);
    /**
     * chooses which shoot method to use based on gate (only false)
     */
    public Command shoot = conditional(
            () -> shooter.isOpen(),
            fastShoot,
            slowShoot
    )
            .requiring(intake, shooter)
            .setPriority(2);
    //*other shooter commands
    /**
     * automatically opens gate based on beam breaks
     */
    public Command handleGate = infinite(() -> {
                if (beamBreaks.getBallCount() == 3 && intakeState != IntakeState.IN) {
                    shooter.openGate();
                } else {
                shooter.closeGate();
                }
            }
    )
            .requiring(shooter)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE);

    /**
     * lifts intake automatically and runs intake based on intakeState
     */
    public Command handleIntake = infinite(
            () -> {
//                if (beamBreaks.getBallCount() == 3){
//                    intake.lift();
//                } else {
//                    intake.lower();
//                }
                switch (intakeState) {
                    case IN:
                        if (beamBreaks.getBallCount() != 3) {
                            intake.spinIn();
                        }
                        break;
                    case OUT:
                        intake.spinOut();
                        break;
                    case OFF:
                        intake.stop();
                        break;
                }
            }
    )
            .requiring(intake)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE);

    public void setIntakeState(IntakeState intakeState) {
        if (this.intakeState != intakeState) {
            this.intakeState = intakeState;
        }
        if (intakeState == IntakeState.OUT){ //this happens continuously, regardless of if it's new or not
            beamBreaks.reset();
        }
    }


    public void initialize(boolean isRed, HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //we can try setting this to manual and see how much loop times improve
        }

        follower = Constants.createFollower(hwMap);
        intake.initialize(hwMap);
        shooter.initialize(hwMap);
        huskyLens.initialize(hwMap);
        beamBreaks.initialize(hwMap);
        limelight.initialize(hwMap);

        //kickstand.init(hwMap);
        this.isRed = isRed;
        if (isRed) {
            goalPose = redGoal;
            hpz = redHpz;
            limelight.setPipeline(0);
        } else {
            goalPose = redGoal.mirror();
            hpz = redHpz.mirror();
            limelight.setPipeline(1);
        }
        follower.update();
    }

    /**
     * gets the distance from the follower's current pose to the goal
     * @return the distance from the robot to the goal
     */
    public double getDistToGoal() {
        double xDiff = follower.getPose().getX() - goalPose.getX();
        double yDiff = follower.getPose().getY() - goalPose.getY();
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    /**
     * updates shooter (autoRPM), beam breaks, limelight, follower, and drive
     * @param forward forward drive command
     * @param right strafe drive command
     * @param clockwise rotate drive command
     */
    public void update(double forward, double right, double clockwise) {
        follower.update();

        limelight.update();

//        shooter.update(getDistToGoal());
        shooter.runWithPIDF(0);

        beamBreaks.updatePrism(isShooting, autoAiming);

        //kickstand.update();

            forwardInput = -forward;
            rightInput = -right;
            rotateInput = -clockwise;

        drive(-forward, -right, -clockwise);
    }

    public double getRealAngleToGoalDeg(){
        double xDiff = goalPose.getX() - follower.getPose().getX();
        double yDiff = goalPose.getY() - follower.getPose().getY();
        double angleFromCoords = Math.toDegrees(Math.atan2(yDiff, xDiff));
        return normalizeAngle(angleFromCoords, false, AngleUnit.DEGREES);
    }
    public double getAngleToSotmGoalDeg(){
        double xDiff = getSotmOffset().getX() - follower.getPose().getX();
        double yDiff = getSotmOffset().getY() - follower.getPose().getY();
        double angleFromCoords = Math.toDegrees(Math.atan2(yDiff, xDiff));
        return normalizeAngle(angleFromCoords, false, AngleUnit.DEGREES);
    }

    /**
     * gets the difference of the follower's angle and the angle it needs to be at to face the goal.
     * @param sotm if this should use the offset shoot-on-the-move goal pose or the real one
     * @return the error in degrees
     */
    public double getOdoGoalAngleErrorDeg(boolean sotm) {
        double targetAngle = sotm ? getAngleToSotmGoalDeg() : getRealAngleToGoalDeg();
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());

        return normalizeAngle(targetAngle - currentHeading, false, AngleUnit.DEGREES);
    }

    public double getAimingPIDFOutput(double angleErrorDeg) {
        PIDController headingPID = new PIDController(headingKP, headingKI, headingKD); //robot todo tune this
        return -1 * (Range.clip((headingPID.calculate(angleErrorDeg) - headingKF * Math.signum(angleErrorDeg)), -1, 1));
    }

    public Pose getSotmOffset(){
        Vector velocity = follower.getVelocity();
        double seconds = /*shooter.secShotTakes.get(getDistToGoal())*/0.5;
        return new Pose(
                goalPose.getX() + velocity.getXComponent(),
                goalPose.getY() + velocity.getYComponent()
        );
    }

    //*Kalman filter stuff (not used)

    double x = 0; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double p = 1; // your initial covariance guess
    double K = 1; // your initial Kalman gain guess

    double x_previous = x;
    double p_previous = p;
    double odoChange = 0;
    double z = 0;
    public Pose filteredOdoPose(Pose rawOdometryPose) {
        odoChange = 1; // Ex: change in position from odometry.
        x = x_previous + odoChange;

        p = p_previous + Q;

        K = p/(p + R);

        z = 1; // Pose Estimate from April Tag / Distance Sensor

        x = x + K * (z - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;
        return new Pose();
    }

    public Pose[] getRobotCorners(Pose robotPose) {
        double half = 9.0; // half of 18 inches
        double x = robotPose.getX();
        double y = robotPose.getY();
        double heading = robotPose.getHeading();

        // Corners relative to center, in robot-local frame (x forward, y left)
        double[][] localCorners = {
                { half,  half}, // front-left
                { half, -half}, // front-right
                {-half, -half}, // back-right
                {-half,  half}  // back-left
        };

        Pose[] corners = new Pose[4];
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        for (int i = 0; i < 4; i++) {
            double lx = localCorners[i][0];
            double ly = localCorners[i][1];

            // Rotate local offset into field frame
            double fieldX = x + (lx * cos - ly * sin);
            double fieldY = y + (lx * sin + ly * cos);

            corners[i] = new Pose(fieldX, fieldY, heading);
        }
        return corners;
    }

    public Vector getFieldRelativeMovement(double forward, double strafe, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double fieldX = forward * cos - strafe * sin;
        double fieldY = forward * sin + strafe * cos;

        Vector movement = new Vector();
        movement.setOrthogonalComponents(fieldX, fieldY);
        return movement;
    }

    public Vector getRobotRelativeMovement(Vector fieldMovement, double heading) {
        double fieldX = fieldMovement.getXComponent();
        double fieldY = fieldMovement.getYComponent();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Inverse rotation (rotate by -heading)
        double localForward = fieldX * cos + fieldY * sin;
        double localStrafe  = -fieldX * sin + fieldY * cos;

        Vector local = new Vector();
        local.setOrthogonalComponents(localForward, localStrafe);
        return local;
    }

    public void drive(double forward, double strafe,  double turn){
        Pose botPose = follower.getPose();
        xMargin = 3 + abs((follower.getVelocity().getXComponent()/Constants.driveConstants.xVelocity) * 11);
        yMargin = 3 + abs((follower.getVelocity().getYComponent()/Constants.driveConstants.xVelocity) * 11);
        if (slowDrive) {
            forward *= 0.2;
            strafe *= 0.2;
            turn *= 0.2;
        }
        Pose[] corners = getRobotCorners(botPose);
        if (isCloseToBump(botPose)) {
            Vector movement = getFieldRelativeMovement(forward, strafe, botPose.getHeading());
            double x = movement.getXComponent();
            double y = movement.getYComponent();

            Pose corner = getClosestCorner(botPose);
            if ((corner.getX() > (BUMP_MIN_X - xMargin)) && (corner.getX() < BUMP_MIN_X)){
                x = min(-((follower.getVelocity().getXComponent()/Constants.driveConstants.xVelocity*4)+0.05), x);
            } else if (corner.getX() < (BUMP_MAX_X + xMargin) && corner.getX() > BUMP_MAX_X) {
                x = max(-((follower.getVelocity().getXComponent()/Constants.driveConstants.xVelocity*4)+0.05), x);
            }
            if (corner.getY() > (BUMP_MIN_Y - yMargin) && corner.getY() < BUMP_MIN_Y){
                y = min(-((follower.getVelocity().getYComponent()/Constants.driveConstants.xVelocity*4)+0.05), y);
            } else if (corner.getY() < (BUMP_MAX_Y + yMargin) && corner.getY() > BUMP_MAX_Y) {
                y = max(-((follower.getVelocity().getYComponent()/Constants.driveConstants.xVelocity*4)+0.05), y);
            }
            movement.setOrthogonalComponents(x, y);
            forward = getRobotRelativeMovement(movement, botPose.getHeading()).getXComponent();
            strafe = getRobotRelativeMovement(movement, botPose.getHeading()).getYComponent();
        }
        if (!follower.isTeleopDrive()) follower.startTeleOpDrive();
        follower.setTeleOpDrive(forward, strafe, turn);
    }

    double getDistFromPoints(Pose start, Pose end) {
        double xDiff = end.getX() - start.getX();
        double yDiff = end.getY() - start.getY();
        return abs(Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)));
    }

    private final double BUMP_MIN_X = 47.75, BUMP_MIN_Y = 53, BUMP_MAX_X = 96.25, BUMP_MAX_Y = 91;
    public double xMargin = 2.5, yMargin = 2.5;

    public boolean isCloseToBump(Pose botPose){
        for (Pose corner : getRobotCorners(botPose)) {
            boolean withinX = (corner.getX() > (BUMP_MIN_X - xMargin) && corner.getX() < (BUMP_MAX_X + xMargin));
            boolean withinY = (corner.getY() > (BUMP_MIN_Y - yMargin) && corner.getY() < (BUMP_MAX_Y + yMargin));
            if (withinX && withinY) {
                return true;
            }
        }
        return false;
    }


    public Pose getClosestCorner(Pose botPose){
        double minDist = 999999;
        Pose closestPose = new Pose();
        for (Pose corner : getRobotCorners(botPose)) {
            boolean withinX = (botPose.getX() > BUMP_MIN_X - xMargin && botPose.getX() < BUMP_MAX_X + xMargin);
            boolean withinY = (botPose.getY() > BUMP_MIN_Y - yMargin && botPose.getY() < BUMP_MAX_Y + yMargin);
            double distToCenter = getDistFromPoints(corner, new Pose(72, 72, 0));
            if (distToCenter < minDist) {
                minDist = distToCenter;
                closestPose = corner;
            }
        }
        return closestPose;
    }




}
