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
    public Pose2d BluStart;
    public static Trajectory BluUpAndOverTrajectory;
    public static Trajectory BluDownAndUnderTrajectory;
    public static Trajectory BluAcrossRampTrajectory;
    public static Trajectory BluOntoRampTrajectory;
    public static Trajectory RedUpAndOverTrajectory;
    public static Trajectory RedDownAndUnderTrajectory;
    public static Trajectory RedAcrossRampTrajectory;
    public static Trajectory RedOntoRampTrajectory;

    public Trajectories(DrivetrainSubsystem drivetrain){

        TrajectoryConfig config = drivetrain.getTrajectoryConfig();
        System.out.println("created trajectory config");

        this.BluStart = new Pose2d(1.678, 2.748, Rotation2d.fromDegrees(180));
        Pose2d BluEnd = new Pose2d(6.7, 2.748, Rotation2d.fromDegrees(90));
        Pose2d BluRamp = new Pose2d(3.922, 2.748, Rotation2d.fromDegrees(90));

        ArrayList<Translation2d> BluUpAndOverWaypoints = new ArrayList<Translation2d>();
        BluUpAndOverWaypoints.add(new Translation2d(1.769, 4.122));
        BluUpAndOverWaypoints.add(new Translation2d(3.3, 4.122));
        BluUpAndOverWaypoints.add(new Translation2d(4.9, 4.122));
        System.out.println(">>>>> Generating Blue up and over");
        this.BluUpAndOverTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BluStart, BluUpAndOverWaypoints, BluEnd, config);

        ArrayList<Translation2d> BluDownAndUnderWaypoints = new ArrayList<Translation2d>();
        BluDownAndUnderWaypoints.add(new Translation2d(2.769, 1.374));
        BluDownAndUnderWaypoints.add(new Translation2d(4.9, 1.374));
        System.out.println(">>>>> Generating Blue down and under");
        this.BluDownAndUnderTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BluStart, BluDownAndUnderWaypoints, BluEnd, config);
        SwerveTrajectoryGenerator.printTrajectory(BluDownAndUnderTrajectory);
        
        ArrayList<Pose2d> BlueAcrossRampWaypoints = new ArrayList<Pose2d>();
        BlueAcrossRampWaypoints.add(BluStart);
        BlueAcrossRampWaypoints.add(BluEnd);
        System.out.println(">>>>> Generating Blue across ramp");
        this.BluAcrossRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BlueAcrossRampWaypoints, config);
        SwerveTrajectoryGenerator.printTrajectory(BluAcrossRampTrajectory);
        
        ArrayList<Pose2d> BluOntoRampWaypoints = new ArrayList<Pose2d>();
        BluOntoRampWaypoints.add(BluEnd);
        BluOntoRampWaypoints.add(BluRamp);
        System.out.println(">>>>> Generating Blue onto ramp");
        this.BluOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BluOntoRampWaypoints, config);
        System.out.println(">>>>> Done Generating Blue trajectories!!");

        /* 
        Pose2d RedStart = new Pose2d(14.533, 2.748, Rotation2d.fromDegrees(0));
        Pose2d RedEnd = new Pose2d(11.34, 2.748, Rotation2d.fromDegrees(90));
        Pose2d RedRamp = new Pose2d(12.618, 2.748, Rotation2d.fromDegrees(90));

        ArrayList<Translation2d> RedUpAndOverWaypoints = new ArrayList<Translation2d>();
        RedUpAndOverWaypoints.add(new Translation2d(13.771, 4.122));
        RedUpAndOverWaypoints.add(new Translation2d(11.64, 4.122));
        this.BluUpAndOverTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RedStart, BluUpAndOverWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedDownAndUnderWaypoints = new ArrayList<Translation2d>();
        RedDownAndUnderWaypoints.add(new Translation2d(13.771, 1.374));
        RedDownAndUnderWaypoints.add(new Translation2d(11.64, 1.374));
        this.RedDownAndUnderTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RedStart, BluDownAndUnderWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedAcrossRampWaypoints = new ArrayList<Translation2d>();
        RedAcrossRampWaypoints.add(new Translation2d(11.34, 2.748));
        this.RedAcrossRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RedStart, BluAcrossRampWaypoints, RedEnd, config);
        ArrayList<Translation2d> RedOntoRampWaypoints = new ArrayList<Translation2d>();
        RedOntoRampWaypoints.add(new Translation2d(12.618, 2.748));
        this.RedOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RedEnd, BluOntoRampWaypoints, RedRamp, config);
*/
    }


    
}   
