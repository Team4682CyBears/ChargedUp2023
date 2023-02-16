package frc.robot.control;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import java.util.ArrayList;

public class Trajectories {
    public Pose2d Node1Position;
    public Pose2d Node5Position;
    public Pose2d Node9Position;
    public Pose2d TrajectoryEndPosition;
    public static Trajectory LeftTrajectory;
    public static Trajectory RightTrajectory;
    public static Trajectory MiddleTrajectory;
    public Trajectory OntoRampTrajectory;
    
    public Trajectories(DrivetrainSubsystem drivetrain){

        TrajectoryConfig config = drivetrain.getTrajectoryConfig();
        System.out.println("created trajectory config");

        this.Node1Position = new Pose2d(1.678, 5.084, Rotation2d.fromDegrees(180));
        this.Node5Position = new Pose2d(1.678, 2.750, Rotation2d.fromDegrees(180));
        this.Node9Position = new Pose2d(1.678, 0.416, Rotation2d.fromDegrees(180));

        this.TrajectoryEndPosition = new Pose2d(5.81, 2.748, Rotation2d.fromDegrees(90));
        Pose2d Ramp = new Pose2d(3.922, 2.748, Rotation2d.fromDegrees(90));
 
        ArrayList<Translation2d> LeftWaypoints = new ArrayList<Translation2d>();
        LeftWaypoints.add(new Translation2d(2.1, 4.67));
        LeftWaypoints.add(new Translation2d(3.7, 4.67));
        LeftWaypoints.add(new Translation2d(5.3, 4.67));
        this.LeftTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node1Position, LeftWaypoints, TrajectoryEndPosition, config);

        ArrayList<Translation2d> RightWaypoints = new ArrayList<Translation2d>();
        RightWaypoints.add(new Translation2d(2.1, .69));
        RightWaypoints.add(new Translation2d(3.7, .69));
        RightWaypoints.add(new Translation2d(5.3, .69));
        this.RightTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node9Position, RightWaypoints, TrajectoryEndPosition, config);
        SwerveTrajectoryGenerator.printTrajectory(RightTrajectory);
        
        ArrayList<Pose2d> MiddleWaypoints = new ArrayList<Pose2d>();
        MiddleWaypoints.add(Node5Position);
        MiddleWaypoints.add(TrajectoryEndPosition);
        this.MiddleTrajectory = SwerveTrajectoryGenerator.generateTrajectory(MiddleWaypoints, config);
        SwerveTrajectoryGenerator.printTrajectory(MiddleTrajectory);
        
        ArrayList<Pose2d> OntoRampWaypoints = new ArrayList<Pose2d>();
        OntoRampWaypoints.add(TrajectoryEndPosition);
        OntoRampWaypoints.add(Ramp);
        this.OntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(OntoRampWaypoints, config);    
    }


    
}   
