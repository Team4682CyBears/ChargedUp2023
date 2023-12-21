// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: CameraSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.SubsystemCollection;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {
  DrivetrainSubsystem drivetrainSubsystem;
  private final double milisecondsInSeconds = 1000.0;
  private final int defaultDoubleArraySize = 7;
  private final int TimestampIndex = 6;
  private final int botPositionXIndex = 0;
  private final int botPositionYIndex = 2;
  private final int botRotationIndex = 5;

  /**
   * a constructor for the camera subsystem class
   * @param subsystems - the subsystem collection
   */
  public CameraSubsystem(SubsystemCollection subsystems) {
    drivetrainSubsystem = subsystems.getDriveTrainSubsystem();
  }

  /**
   * A method to run during periodic for the camera subsystem
   * it reads tags and updates the estimated position in the drivetrain subsystem
   */
  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tagId = table.getEntry("tid").getDouble(0);
    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[defaultDoubleArraySize]);
    Double timestamp = Timer.getFPGATimestamp() - (botpose[TimestampIndex]/milisecondsInSeconds);
    Translation2d botTranslation = new Translation2d(botpose[botPositionXIndex], botpose[botPositionYIndex]);
    Rotation2d botYaw = new Rotation2d(botpose[botRotationIndex]);
    Pose2d RealBotpos = new Pose2d(botTranslation, botYaw);

    // the tag ID is -1 if there is no tag in view
    if (tagId != -1){
      drivetrainSubsystem.AddVisionMeasurement(RealBotpos, timestamp);
    }
  }
}

