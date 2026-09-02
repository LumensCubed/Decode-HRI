package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.normalizeAngle;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Util {
    /**
     * Finds the angle between 2 Poses, from pi to -pi.
     * @param start the first pose (e.g. center of a circle)
     * @param end the other pose (e.g. a point on the circle)
     * @return the angle between the line at 0 rad and the line between the 2 points in radians
     */
    public static double getAbsoluteAngleRad(Pose start, Pose end) {
        double xDiff = end.getX() - start.getX();
        double yDiff = end.getY() - start.getY();
        double angleFromCoords = Math.atan2(yDiff, xDiff);
        return normalizeAngle(angleFromCoords, false, AngleUnit.RADIANS);
    }

    /**
     * Finds the angle between 2 Poses from pi to -pi with an offset
     * (e.g. robot heading to find the difference between the desired heading and the current heading).
     * @param start the first pose (e.g. center of a circle)
     * @param end the other pose (e.g. a point on the circle)
     * @return the angle between the line at 0 rad and the line between the 2 points in radians
     */
    public static double getOffsetAngleRad(Pose start, Pose end, double startHeading) {
        double absAngle = getAbsoluteAngleRad(start, end);
        return normalizeAngle(startHeading + absAngle, false, AngleUnit.RADIANS);
    }
}
