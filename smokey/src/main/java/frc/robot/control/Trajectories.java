package frc.robot.control;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.ArrayList;

public class Trajectories {
    Trajectory BluUpAndOverTrajectory;
    Trajectory BluDownAndUnderTrajectory;
    Trajectory BluAcrossRampTrajectory;
    Trajectory BluOntoRampTrajectory;
    Trajectory RedUpAndOverTrajectory;
    Trajectory RedDownAndUnderTrajectory;
    Trajectory RedAcrossRampTrajectory;
    Trajectory RedOntoRampTrajectory;

    TrajectoryConfig config = new TrajectoryConfig(Constants.TrajectoryMaxVelocity, Constants.TrajectoryMaxAcceleration);

    public Trajectories(){

        Pose2d BluStart = new Pose2d(2.007, 2.748, Rotation2d.fromDegrees(0));
        Pose2d BluEnd = new Pose2d(5.2, 2.748, Rotation2d.fromDegrees(90));
        Pose2d BluRamp = new Pose2d(3.922, 2.748, Rotation2d.fromDegrees(90));

        ArrayList<Translation2d> BluUpAndOverWaypoints = new ArrayList<Translation2d>();
        BluUpAndOverWaypoints.add(new Translation2d(2.769, 4.122));
        BluUpAndOverWaypoints.add(new Translation2d(4.9, 4.122));
        this.BluUpAndOverTrajectory = TrajectoryGenerator.generateTrajectory(BluStart, BluUpAndOverWaypoints, BluEnd, config);
        ArrayList<Translation2d> BluDownAndUnderWaypoints = new ArrayList<Translation2d>();
        BluDownAndUnderWaypoints.add(new Translation2d(2.769, 1.374));
        BluDownAndUnderWaypoints.add(new Translation2d(4.9, 1.374));
        this.BluDownAndUnderTrajectory = TrajectoryGenerator.generateTrajectory(BluStart, BluDownAndUnderWaypoints, BluEnd, config);
        ArrayList<Translation2d> BluAcrossRampWaypoints = new ArrayList<Translation2d>();
        BluAcrossRampWaypoints.add(new Translation2d(5.2, 2.748));
        this.BluAcrossRampTrajectory = TrajectoryGenerator.generateTrajectory(BluStart, BluAcrossRampWaypoints, BluEnd, config);
        ArrayList<Translation2d> BluOntoRampWaypoints = new ArrayList<Translation2d>();
        BluOntoRampWaypoints.add(new Translation2d(3.922, 2.748));
        this.BluOntoRampTrajectory = TrajectoryGenerator.generateTrajectory(BluEnd, BluOntoRampWaypoints, BluRamp, config);

        Pose2d RedStart = new Pose2d(14.533, 2.748, Rotation2d.fromDegrees(0));
        Pose2d RedEnd = new Pose2d(11.34, 2.748, Rotation2d.fromDegrees(90));
        Pose2d RedRamp = new Pose2d(12.618, 2.748, Rotation2d.fromDegrees(90));

        ArrayList<Translation2d> RedUpAndOverWaypoints = new ArrayList<Translation2d>();
        RedUpAndOverWaypoints.add(new Translation2d(13.771, 4.122));
        RedUpAndOverWaypoints.add(new Translation2d(11.64, 4.122));
        this.BluUpAndOverTrajectory = TrajectoryGenerator.generateTrajectory(RedStart, BluUpAndOverWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedDownAndUnderWaypoints = new ArrayList<Translation2d>();
        RedDownAndUnderWaypoints.add(new Translation2d(13.771, 1.374));
        RedDownAndUnderWaypoints.add(new Translation2d(11.64, 1.374));
        this.RedDownAndUnderTrajectory = TrajectoryGenerator.generateTrajectory(RedStart, BluDownAndUnderWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedAcrossRampWaypoints = new ArrayList<Translation2d>();
        RedAcrossRampWaypoints.add(new Translation2d(11.34, 2.748));
        this.RedAcrossRampTrajectory = TrajectoryGenerator.generateTrajectory(RedStart, BluAcrossRampWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedOntoRampWaypoints = new ArrayList<Translation2d>();
        RedOntoRampWaypoints.add(new Translation2d(12.618, 2.748));
        this.RedOntoRampTrajectory = TrajectoryGenerator.generateTrajectory(RedEnd, BluOntoRampWaypoints, RedRamp, config);

    }


    
}   
