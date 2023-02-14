// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class with trajcetories for testing swerve drives.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * A class with trajcetories for testing swerve drives.
 */
public class TestTrajectories {
    public Trajectory traverseSimpleForward;
    public Trajectory traverseSimpleLeft;
    public Trajectory traverseTurn270;
    public Trajectory turn90;
    public Trajectory traverseForwardArc;
    public Trajectory traverseBackwardArc;
    public Pose2d traverseBackwardArcStartPosition = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0.0));
    private TrajectoryConfig config; 

    /**
    * constructs trajcetories for testing swerve drives.
    */
    public TestTrajectories(TrajectoryConfig config){
        this.config = config; 
        traverseSimpleForward = buildTraverseSimpleForward();
        traverseSimpleLeft = buildTraverseSimpleLeft();
        traverseTurn270 = buildTraverseTurn270();
        turn90 = buildTurn90();
        traverseForwardArc = buildTraverseForwardArc();
        traverseBackwardArc = buildTraverseBackwardArc();
    }

    private Trajectory buildTraverseSimpleForward(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0)));
    
        System.out.println(">>>>>>>>>>>>>>>> Generating Traverse Simple Forward");
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(waypoints, config); 
        return t;
      }
    
      private Trajectory buildTraverseSimpleLeft(){
        System.out.println(">>>>>>>>>>>>>>>> Generating Traverse Simple Right");
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0)));
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(waypoints, config); 
        return t;
      }
    
      private Trajectory buildTraverseTurn270(){
        System.out.println(">>>>>>>>>>>>>>>> Generating Traverse Turn 270");
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(-90)));
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(waypoints, config); 
        return t;
      }
    
      // Test purely rotational trajectory.  
      private Trajectory buildTurn90(){
        System.out.println(">>>>>>>>>>>>>>>> Generating Turn 90");
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90)));
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(waypoints, config); 
        return t;
      }
    
      private Trajectory buildTraverseForwardArc(){
        System.out.println(">>>>>>>>>>>>>>>> Building Traverse Forward Arc");
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        Pose2d end = this.traverseBackwardArcStartPosition;
    
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0.5, 0.25));
        interiorWaypoints.add(new Translation2d(1.0, 0.50));
        interiorWaypoints.add(new Translation2d(1.5, 0.25));
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config); 
        return t;
      }
    
      private Trajectory buildTraverseBackwardArc(){
        System.out.println(">>>>>>>>>>>>>>>> Building Traverse Backward Arc");
        Pose2d start = this.traverseBackwardArcStartPosition;
        Pose2d end = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(1.5, 0.25));
        interiorWaypoints.add(new Translation2d(1.0, 0.50));
        interiorWaypoints.add(new Translation2d(0.5, 0.25));
        Trajectory t = SwerveTrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config); 
        return t;
      }
    
    
}
