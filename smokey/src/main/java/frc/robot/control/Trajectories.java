package frc.robot.control;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;

public class Trajectories {
    private Pose2d Node1Position;
    private Pose2d Node2Position;
    private Pose2d Node4Position;
    private Pose2d Node6Position;
    private Pose2d Node5Position;
    private Pose2d Node8Position;
    private Pose2d Node9Position;
    private Pose2d InfrontOfRampPosition;
    private Pose2d Red4InfrontOfRampPosition;
    private Pose2d Blue6InfrontOfRampPosition; 
    private Pose2d RampFarWaypoint;
    private Pose2d Red4RampFarWaypoint;
    private Pose2d Blue6RampFarWaypoint;
    private Trajectory LeftTrajectory;
    private Trajectory Node2Trajectory;
    private Trajectory RightTrajectory;
    private Trajectory Node8Trajectory;
    private Trajectory MiddleTrajectoryPart1;
    private Trajectory MiddleTrajectoryPart2;
    private Trajectory Red4TrajectoryPart1;
    private Trajectory Red4TrajectoryPart2;
    private Trajectory Blue6TrajectoryPart1;
    private Trajectory Blue6TrajectoryPart2;
    private Trajectory LeftToOntoRampTrajectory;
    private Trajectory RightToOntoRampTrajectory;
    private Trajectory BehindToOntoRampTrajectory;
    private Trajectory Red4BehindToOntoRampTrajectory;
    private Trajectory Blue6BehindToOntoRampTrajectory;
    private Trajectory DirectToRampTrajectory;
    private Trajectory MiddlePathBehindToOntoRampTrajectory;
    private Trajectory Red4PathBehindToOntoRampTrajectory;
    private Trajectory Blue6PathBehindToOntoRampTrajectory;
    
    public SwerveTrajectoryConfig config;
    public SwerveTrajectoryConfig firstSegmentConfig;
    public SwerveTrajectoryConfig middleSegmentConfig;
    public SwerveTrajectoryConfig lastSegmentConfig;

    private DrivetrainSubsystem drivetrain;

    public Trajectories(DrivetrainSubsystem drivetrain){
        this.drivetrain = drivetrain; 

        config = drivetrain.getTrajectoryConfig();
        // trajectory config with a fast starting velocity for ramp driving. 
        // have to get a new config so that changes to this one don't affect the original
        SwerveTrajectoryConfig fastConfig = drivetrain.getTrajectoryConfig();
        fastConfig.setStartVelocity(fastConfig.getMaxVelocity() * 0.6); // less than max speed
        // trajectory config that will start limit to a slow velocity for driving off ramp
        double offOfRampSpeed = 1.25; 
        SwerveTrajectoryConfig offOfRampConfig = new SwerveTrajectoryConfig(
            offOfRampSpeed, 
            config.getMaxAcceleration(),
            config.getMaxRotationalVelocity(),
            config.getMaxRotationalAcceleration());
        // trajectory configs for joining trajectory segments together without slowing down between segments
        double trajectoryJoinSpeed = config.getMaxVelocity() * 0.4;
        firstSegmentConfig = drivetrain.getTrajectoryConfig();
        firstSegmentConfig.setEndVelocity(trajectoryJoinSpeed);
        middleSegmentConfig = drivetrain.getTrajectoryConfig();
        middleSegmentConfig.setStartVelocity(trajectoryJoinSpeed).setEndVelocity(trajectoryJoinSpeed);
        lastSegmentConfig = drivetrain.getTrajectoryConfig();
        lastSegmentConfig.setStartVelocity(trajectoryJoinSpeed);

        this.Node1Position = new Pose2d(1.678, 4.994, Rotation2d.fromDegrees(180));
        this.Node2Position = new Pose2d(1.678, 4.433, Rotation2d.fromDegrees(180));
        this.Node4Position = new Pose2d(1.678, 3.311, Rotation2d.fromDegrees(180));
        this.Node5Position = new Pose2d(1.678, 2.750, Rotation2d.fromDegrees(180));
        this.Node6Position = new Pose2d(1.678, 2.189, Rotation2d.fromDegrees(180));
        this.Node8Position = new Pose2d(1.678, 1.067, Rotation2d.fromDegrees(180));
        this.Node9Position = new Pose2d(1.678, 0.506, Rotation2d.fromDegrees(180));
        this.InfrontOfRampPosition = new Pose2d(2.0, 2.41, Rotation2d.fromDegrees(0)); 
        this.Red4InfrontOfRampPosition = new Pose2d(2.0, 3.07, Rotation2d.fromDegrees(0)); 
        this.Blue6InfrontOfRampPosition = new Pose2d(2.0, 2.41, Rotation2d.fromDegrees(0)); 
        
        // There is slippage getting onto ramp, so we need to overshoot the center 
        // to ensure the robot gets far enough onto the ramp.   
        this.RampFarWaypoint = new Pose2d(4.122, 2.41, Rotation2d.fromDegrees(0)); 
        this.Red4RampFarWaypoint = new Pose2d(4.122, 3.07, Rotation2d.fromDegrees(0)); 
        this.Blue6RampFarWaypoint = new Pose2d(4.122, 2.41, Rotation2d.fromDegrees(0)); 

        // behind ramp position for node 5 path
        Pose2d Red4PathOverRampPosition = new Pose2d(6.2 - Units.inchesToMeters(6), 3.07, Rotation2d.fromDegrees(180));
        Pose2d Red4PathRampNearWaypoint = new Pose2d(4.3 - Units.inchesToMeters(6), 3.07, Rotation2d.fromDegrees(180));
        
        // behind ramp position for node 5 path
        Pose2d Blue6PathOverRampPosition = new Pose2d(6.2 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));
        Pose2d Blue6PathRampNearWaypoint = new Pose2d(4.3 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));

        // behind ramp position for node 5 path
        Pose2d MiddlePathOverRampPosition = new Pose2d(6.2 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));
        Pose2d MiddlePathRampNearWaypoint = new Pose2d(4.3 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));

        Pose2d RampNearWaypoint = new Pose2d(3.34, 2.41, Rotation2d.fromDegrees(180));
        // behind ramp position for node 1,2,8,9 paths
        Pose2d BehindTrajectoryEndPosition = new Pose2d(5.27, 2.41, Rotation2d.fromDegrees(180));
        
        // Left waypoints drive from Node 1 or 2 to a location out of the community
        ArrayList<Translation2d> LeftWaypoints = new ArrayList<Translation2d>();
        LeftWaypoints.add(new Translation2d(2.1, 4.67));
        LeftWaypoints.add(new Translation2d(3.7, 4.67));
        Pose2d LeftTrajectoryEndPosition = new Pose2d(5.3, 4.67, Rotation2d.fromDegrees(0));
        this.LeftTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node1Position, LeftWaypoints, LeftTrajectoryEndPosition, config);
        this.Node2Trajectory = SwerveTrajectoryGenerator.generateTrajectory(Node2Position, LeftWaypoints, LeftTrajectoryEndPosition, config);
        
        // Right waypoints drive from Node 8 or 9 to a location out of the community
        ArrayList<Translation2d> RightWaypoints = new ArrayList<Translation2d>();
        RightWaypoints.add(new Translation2d(2.1, .69));
        RightWaypoints.add(new Translation2d(3.7, .69));
        Pose2d RightTrajectoryEndPosition = new Pose2d(5.3, .69, Rotation2d.fromDegrees(0));
        this.RightTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node9Position, RightWaypoints, RightTrajectoryEndPosition, config);
        this.Node8Trajectory = SwerveTrajectoryGenerator.generateTrajectory(Node8Position, RightWaypoints, RightTrajectoryEndPosition, config);

        // To drive from left or right onto ramp, use a common central waypoint
        ArrayList<Translation2d> BehindToRampWaypoints = new ArrayList<Translation2d>();
        BehindToRampWaypoints.add(new Translation2d(5.81, 2.748));
        this.LeftToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(LeftTrajectoryEndPosition, BehindToRampWaypoints, BehindTrajectoryEndPosition, config);
        this.RightToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RightTrajectoryEndPosition, BehindToRampWaypoints, BehindTrajectoryEndPosition, config);
        
        // Drive onto ramp from behind
        ArrayList<Pose2d> BehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        BehindToOntoRampWaypoints.add(BehindTrajectoryEndPosition);
        BehindToOntoRampWaypoints.add(RampNearWaypoint);
        // use fastConfig for this trajectory 
        this.BehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BehindToOntoRampWaypoints, fastConfig); 

        this.LeftToOntoRampTrajectory = this.LeftToOntoRampTrajectory.concatenate(BehindToOntoRampTrajectory);
        this.RightToOntoRampTrajectory = this.RightToOntoRampTrajectory.concatenate(BehindToOntoRampTrajectory);

        /// NODE 555555 ROUTINES /////
        // Drive onto ramp from in front 
        ArrayList<Pose2d> InfrontToOntoRampWaypoints = new ArrayList<Pose2d>();
        InfrontToOntoRampWaypoints.add(InfrontOfRampPosition);
        InfrontToOntoRampWaypoints.add(RampFarWaypoint);
        // use fastConfig for this trajectory 
        Trajectory InfrontToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(InfrontToOntoRampWaypoints, fastConfig);

        ArrayList<Pose2d> Node5ToFrontOfRampWaypoints = new ArrayList<Pose2d>();
        Node5ToFrontOfRampWaypoints.add(Node5Position);
        Node5ToFrontOfRampWaypoints.add(InfrontOfRampPosition);
        Trajectory Node5ToFrontOfRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node5ToFrontOfRampWaypoints, config);
        
        this.DirectToRampTrajectory = Node5ToFrontOfRampTrajectory.concatenate(InfrontToOntoRampTrajectory);
        
        // Construct the middle up and over ramp trajectory
        ArrayList<Pose2d> MiddleWaypoints = new ArrayList<Pose2d>();
        MiddleWaypoints.add(RampFarWaypoint);
        MiddleWaypoints.add(MiddlePathOverRampPosition);
        Trajectory RampToBehindRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(MiddleWaypoints, offOfRampConfig);
        // Drive onto ramp from behind
        ArrayList<Pose2d> MiddlePathBehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        MiddlePathBehindToOntoRampWaypoints.add(MiddlePathOverRampPosition);
        MiddlePathBehindToOntoRampWaypoints.add(MiddlePathRampNearWaypoint);
        // use fastConfig for this trajectory 
        this.MiddlePathBehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(MiddlePathBehindToOntoRampWaypoints, fastConfig); 

        this.MiddleTrajectoryPart1 = Node5ToFrontOfRampTrajectory
            .concatenate(InfrontToOntoRampTrajectory);
        this.MiddleTrajectoryPart2 = RampToBehindRampTrajectory;

        /// NODE 4444444 ROUTINES /////
        // Drive onto ramp from in front 
        ArrayList<Pose2d> Red4InfrontToOntoRampWaypoints = new ArrayList<Pose2d>();
        Red4InfrontToOntoRampWaypoints.add(Red4InfrontOfRampPosition);
        Red4InfrontToOntoRampWaypoints.add(Red4RampFarWaypoint);
        // use fastConfig for this trajectory 
        Trajectory Red4InfrontToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Red4InfrontToOntoRampWaypoints, fastConfig);

        ArrayList<Pose2d> Node4ToFrontOfRampWaypoints = new ArrayList<Pose2d>();
        Node4ToFrontOfRampWaypoints.add(Node4Position);
        Node4ToFrontOfRampWaypoints.add(Red4InfrontOfRampPosition);
        Trajectory Node4ToFrontOfRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node4ToFrontOfRampWaypoints, config);
        
        // Construct the middle up and over ramp trajectory
        ArrayList<Pose2d> Red4Waypoints = new ArrayList<Pose2d>();
        Red4Waypoints.add(Red4RampFarWaypoint);
        Red4Waypoints.add(Red4PathOverRampPosition);
        Trajectory Red4RampToBehindRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Red4Waypoints, offOfRampConfig);
        // Drive onto ramp from behind
        ArrayList<Pose2d> Red4PathBehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        Red4PathBehindToOntoRampWaypoints.add(Red4PathOverRampPosition);
        Red4PathBehindToOntoRampWaypoints.add(Red4PathRampNearWaypoint);
        // use fastConfig for this trajectory 
        this.Red4PathBehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Red4PathBehindToOntoRampWaypoints, fastConfig); 

        this.Red4TrajectoryPart1 = Node4ToFrontOfRampTrajectory
            .concatenate(Red4InfrontToOntoRampTrajectory);
        this.Red4TrajectoryPart2 = Red4RampToBehindRampTrajectory;

        System.out.println("RED 4");

        SwerveTrajectoryGenerator.printTrajectory(Red4PathBehindToOntoRampTrajectory);
        System.out.println("");
        SwerveTrajectoryGenerator.printTrajectory(Red4TrajectoryPart1);
        System.out.println("");
        SwerveTrajectoryGenerator.printTrajectory(Red4TrajectoryPart2);
        
                /// NODE 66666666 ROUTINES /////
        // Drive onto ramp from in front 
        ArrayList<Pose2d> Blue6InfrontToOntoRampWaypoints = new ArrayList<Pose2d>();
        Blue6InfrontToOntoRampWaypoints.add(Blue6InfrontOfRampPosition);
        Blue6InfrontToOntoRampWaypoints.add(Blue6RampFarWaypoint);
        // use fastConfig for this trajectory 
        Trajectory Blue6InfrontToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Blue6InfrontToOntoRampWaypoints, fastConfig);

        ArrayList<Pose2d> Node6ToFrontOfRampWaypoints = new ArrayList<Pose2d>();
        Node6ToFrontOfRampWaypoints.add(Node6Position);
        Node6ToFrontOfRampWaypoints.add(Blue6InfrontOfRampPosition);
        Trajectory Node6ToFrontOfRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node6ToFrontOfRampWaypoints, config);
        
        // Construct the middle up and over ramp trajectory
        ArrayList<Pose2d> Blue6Waypoints = new ArrayList<Pose2d>();
        Blue6Waypoints.add(Blue6RampFarWaypoint);
        Blue6Waypoints.add(Blue6PathOverRampPosition);
        Trajectory Blue6RampToBehindRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Blue6Waypoints, offOfRampConfig);
        // Drive onto ramp from behind
        ArrayList<Pose2d> Blue6PathBehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        Blue6PathBehindToOntoRampWaypoints.add(Blue6PathOverRampPosition);
        Blue6PathBehindToOntoRampWaypoints.add(Blue6PathRampNearWaypoint);
        // use fastConfig for this trajectory 
        this.Blue6PathBehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Blue6PathBehindToOntoRampWaypoints, fastConfig); 

        this.Blue6TrajectoryPart1 = Node6ToFrontOfRampTrajectory
            .concatenate(Blue6InfrontToOntoRampTrajectory);
        this.Blue6TrajectoryPart2 = Blue6RampToBehindRampTrajectory;

        System.out.println("BLUE 6");

        SwerveTrajectoryGenerator.printTrajectory(Blue6PathBehindToOntoRampTrajectory);
        System.out.println("");
        SwerveTrajectoryGenerator.printTrajectory(Blue6TrajectoryPart1);
        System.out.println("");
        SwerveTrajectoryGenerator.printTrajectory(Blue6TrajectoryPart2);

    }

    public SwerveTrajectoryConfig getConfig() {
        return config;
    }
    
    public Trajectory getBehindToOntoRampTrajectory() {
        return BehindToOntoRampTrajectory;
    }

    public Trajectory getDirectToRampTrajectory() {
        return DirectToRampTrajectory;
    }

    public SwerveTrajectoryConfig getFirstSegmentConfig() {
        return firstSegmentConfig;
    }

    public Pose2d getInfrontOfRampPosition() {
        return InfrontOfRampPosition;
    }

    public SwerveTrajectoryConfig getLastSegmentConfig() {
        return lastSegmentConfig;
    }

    public Trajectory getLeftToOntoRampTrajectory() {
        return LeftToOntoRampTrajectory;
    }

    public Trajectory getLeftTrajectory() {
        return LeftTrajectory;
    }

    public Trajectory getMiddlePathBehindToOntoRampTrajectory() {
        return MiddlePathBehindToOntoRampTrajectory;
    }

    public Trajectory getRed4PathBehindToOntoRampTrajectory() {
        return Red4PathBehindToOntoRampTrajectory;
    }

    public Trajectory getBlue6PathBehindToOntoRampTrajectory() {
        return Blue6PathBehindToOntoRampTrajectory;
    }

    public SwerveTrajectoryConfig getMiddleSegmentConfig() {
     
        return middleSegmentConfig;
    }
    
    public Trajectory getMiddleTrajectoryPart1() {
        return MiddleTrajectoryPart1;
    }

    public Trajectory getMiddleTrajectoryPart2() {
        return MiddleTrajectoryPart2;
    }

    public Trajectory getRed4TrajectoryPart1() {
        return Red4TrajectoryPart1;
    }

    public Trajectory getRed4TrajectoryPart2() {
        return Red4TrajectoryPart2;
    }

    public Trajectory getBlue6TrajectoryPart1() {
        return Blue6TrajectoryPart1;
    }

    public Trajectory getBlue6TrajectoryPart2() {
        return Blue6TrajectoryPart2;
    }

    public Pose2d getNode1Position() {
        return Node1Position;
    }

    public Pose2d getNode2Position() {
        return Node2Position;
    }

    public Trajectory getNode2Trajectory() {
        return Node2Trajectory;
    }

    public Pose2d getNode4Position() {
        return Node4Position;
    }

    public Pose2d getNode5Position() {
        return Node5Position;
    }

    public Pose2d getNode6Position() {
        return Node6Position;
    }

    public Pose2d getNode8Position() {
        return Node8Position;
    }

    public Trajectory getNode8Trajectory() {
        return Node8Trajectory;
    }

    public Pose2d getNode9Position() {
        return Node9Position;
    }

    public Pose2d getRampFarWaypoint() {
        return RampFarWaypoint;
    }
    
    public Trajectory getRightToOntoRampTrajectory() {
        return RightToOntoRampTrajectory;
    }

    public Trajectory getRightTrajectory() {
        return RightTrajectory;
    }
}   
