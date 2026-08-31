package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants { //! todo update these, these are old and kinda cooked
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.32)
            .forwardZeroPowerAcceleration(-23.787723000372143)
            .lateralZeroPowerAcceleration(-54.96120147928493)
//            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.04, 0.10749252684655168, 0.001156584908469573))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0.002,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.04,0.025))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.02 ))
            .centripetalScaling(0)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)     //Global Max speed for Pedro Pathing
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(71.85797936146653)
            .yVelocity(52.30964192067544)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.48)//17.8 forward by 18.2 width, 5.5 from back edge
            .strafePodX(-6.84)//8 5/8 off of right edge
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}