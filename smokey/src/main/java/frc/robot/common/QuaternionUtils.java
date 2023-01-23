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
        final Quaternion p = new Quaternion(0.0d, q.getX(), q.getY(), q.getZ());
        final Quaternion r = q.times(p.times(q.inverse()));
        return new Translation3d(r.getX(), r.getY(), r.getZ());
    }

    public static Translation2d getAngleOfSteepestAscent(Quaternion robotPose){
        final Translation3d xyPlaneNormal = new Translation3d(0.0d, 0.0d, 1.0d);
        final Translation3d robotPoseNormal = rotateByQuaternion(xyPlaneNormal, robotPose);
        final Translation3d w = cross(xyPlaneNormal, robotPoseNormal);
        final Translation3d steepestDescent = cross(w, robotPoseNormal);
        return new Translation2d(-1.0 * steepestDescent.getX(), -1.0 * steepestDescent.getY());
    }

}
