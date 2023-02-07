package frc.robot.common;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class SwerveTrajectoryGenerator {

    /**
     * Generates trajectories for swerver drives.  The translation motions of the trajectory is 
     * unconstrained by the initial and final robot pose.  All trajectory states' rotations are then overridden 
     * by the desired final pose.  The profiled PID controller in the holonomic drive controller
     * handles smoothing out the trajectories. 
     * @param start
     * @param interiorWaypoints
     * @param end
     * @param config
     * @return
     */
    public static Trajectory generateTrajectory(Pose2d start, ArrayList<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config){
        Rotation2d origStartingAngle = start.getRotation();
        Rotation2d origEndingAngle = end.getRotation();
        Rotation2d newStartingAngle = interiorWaypoints.get(0).minus(start.getTranslation()).getAngle();
        Rotation2d newEndingAngle = end.getTranslation().minus(interiorWaypoints.get(interiorWaypoints.size()-1)).getAngle();

        start = new Pose2d(start.getTranslation(), newStartingAngle);
        end = new Pose2d(end.getTranslation(), newEndingAngle);
        Trajectory t = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config); 
        // fix up the rotations in every state to always be the final rotation. 
        List<Trajectory.State> states = t.getStates();
        states = OverrideStateRotations(states, origEndingAngle);
        //Calaculate time for rotational trapezoidal profile. 
        double rotationTime = CalculateRotationTime(config, origStartingAngle, origEndingAngle);
        if (rotationTime > t.getTotalTimeSeconds()){
            // Add a final state with time to complete the full rotation. 
            State finalState = states.get(states.size()-1);
            State newFinalState = new State(rotationTime, finalState.velocityMetersPerSecond, 
            finalState.accelerationMetersPerSecondSq, finalState.poseMeters, finalState.curvatureRadPerMeter);
            states.add(newFinalState);
        }
        
        return new Trajectory(states);
    }

    // make a new generateTrajectory for Pose2d trajectories
    public static Trajectory generateTrajectory(ArrayList<Pose2d> waypoints, TrajectoryConfig config){
        int len = waypoints.size();
        Rotation2d origStartingAngle = waypoints.get(0).getRotation();
        Rotation2d origEndingAngle = waypoints.get(len-1).getRotation();
        Rotation2d newStartingAngle = waypoints.get(1).getTranslation().minus(waypoints.get(0).getTranslation()).getAngle();
        Rotation2d newEndingAngle = waypoints.get(len-1).getTranslation().minus(waypoints.get(len-2).getTranslation()).getAngle();

        waypoints.set(0, new Pose2d(waypoints.get(0).getTranslation(), newStartingAngle));
        waypoints.set(len-1, new Pose2d(waypoints.get(len-1).getTranslation(), newEndingAngle));
        Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config); 
        // fix up the rotations in every state to always be the final rotation. 
        List<Trajectory.State> states = t.getStates();
        states = OverrideStateRotations(states, origEndingAngle);
        //Calaculate time for rotational trapezoidal profile. 
        double rotationTime = CalculateRotationTime(config, origStartingAngle, origEndingAngle);
        if (rotationTime > t.getTotalTimeSeconds()){
            // Add a final state with time to complete the full rotation. 
            State finalState = states.get(states.size()-1);
            State newFinalState = new State(rotationTime, finalState.velocityMetersPerSecond, 
            finalState.accelerationMetersPerSecondSq, finalState.poseMeters, finalState.curvatureRadPerMeter);
            states.add(newFinalState);
        }
        
        return new Trajectory(states);
    }   

    public void printTrajectory(Trajectory t){
        for (int i = 0; i < t.getStates().size(); i++) {
            System.out.println(t.getStates().get(i));
        }
    }


    private static List<Trajectory.State> OverrideStateRotations(List<Trajectory.State> states, Rotation2d finalAngle){
        for (int i = 0; i < states.size(); i++) {
            State s = states.get(i);
            s.poseMeters = new Pose2d(s.poseMeters.getTranslation(), finalAngle);
            states.set(i, s); 
        }
        return states;
    }

    private static double CalculateRotationTime(TrajectoryConfig config, Rotation2d startAngle, Rotation2d endAngle){
        TrapezoidProfile.Constraints thetaProfileConstraints = new TrapezoidProfile.
        Constraints(config.getMaxVelocity(),config.getMaxAcceleration());
        TrapezoidProfile thetaProfile = new TrapezoidProfile(
            thetaProfileConstraints, 
            new TrapezoidProfile.State(endAngle.getRadians(), 0.0),
            new TrapezoidProfile.State(startAngle.getRadians(), 0.0));
        return thetaProfile.totalTime();
    }
}
