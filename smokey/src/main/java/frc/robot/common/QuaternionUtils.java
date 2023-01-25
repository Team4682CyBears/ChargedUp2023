package frc.robot.common;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class QuaternionUtils{

    public static Translation3d cross(Translation3d a, Translation3d b){
        return new Translation3d(
            (-1 * b.getY() * a.getZ()) + (b.getZ() * a.getY()),
            (b.getX() * a.getZ()) - (b.getZ() * a.getX()),
            (-1 * b.getX() * a.getY()) + (b.getY() * a.getX()));
    }

    public static Translation3d rotateByQuaternion(Translation3d vec, Quaternion q){
        // create a quaternion from the vector
        final Quaternion p = new Quaternion(0.0d, vec.getX(), vec.getY(), vec.getZ());
        final Quaternion r = q.times(p).times(q.inverse());
        return new Translation3d(r.getX(), r.getY(), r.getZ());
    }

    /**
    public static Translation2d getAngleOfSteepestAscent(Quaternion robotPose){
        final Translation3d xyPlaneNormal = new Translation3d(0.0d, 0.0d, 1.0d);
        final Translation3d robotPoseNormal = rotateByQuaternion(xyPlaneNormal, robotPose);
        final Translation3d w = cross(xyPlaneNormal, robotPoseNormal);
        final Translation3d steepestDescent = cross(w, robotPoseNormal);
        // TODO remove print statements once this is tested on robot
        System.out.println("****Running Angle of Steepest Ascent****");
        System.out.println("robotPose: " + robotPose);
        System.out.println("robotPoseNormal: " + robotPoseNormal);
        System.out.println("w: " + w);
        System.out.println("steepestDescent: " + steepestDescent);
        return new Translation2d(-1.0 * steepestDescent.getX(), -1.0 * steepestDescent.getY());
    }
    */

    /**
     * Return a vector represting the angle of steepest ascent given pitch, yaw, roll
     * @param robotPose Translation3d. x is pitch, y is roll, z is yaw
     * @return Translation2d - the x,y vector specifying the direction of ascent
     */
    public static Translation2d getAngleOfSteepestAscent(Translation3d robotPose)
    {
        // Move X proportional to the sin of the roll (rotation about y)
        // Move Y proportional to the sin or the pitch (rotation about x)
        return new Translation2d(
            Math.sin(Math.toRadians(robotPose.getY())),
            Math.sin(Math.toRadians(robotPose.getX())));

    }
}
