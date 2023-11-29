// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: CameraSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.SubsystemCollection;

public class CameraSubsystem extends SubsystemBase {
  DrivetrainSubsystem drivetrainSubsystem;
  double storedtid = -1;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(SubsystemCollection subsystems) {
    drivetrainSubsystem = subsystems.getDriveTrainSubsystem();
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tid = table.getEntry("tid").getDouble(0);
    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[7]);
    Double timestamp = Timer.getFPGATimestamp() - (botpose[6]/1000.0);
    Translation2d botTranslation = new Translation2d(botpose[0], botpose[2]);
    Rotation2d botYaw = new Rotation2d(botpose[5]);
    Pose2d RealBotpos = new Pose2d(botTranslation, botYaw);

    if (tid != -1){
      drivetrainSubsystem.AddVisionMeasurement(RealBotpos, timestamp);
    }
  }
}

