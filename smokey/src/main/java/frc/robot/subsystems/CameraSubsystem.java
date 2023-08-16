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
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CameraSubsystem extends SubsystemBase {

  float storedtid = -1;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new ExampleSubsystem. */
  public CameraSubsystem() {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry botpose = table.getEntry("botpose");
    Double timestamp = Timer.getFPGATimestamp() - (botpose[6]/1000.0);
    if (tid != -1 && storedtid != tid){
      DrivetrainSubsystem.AddVisionMeasurement(botpose, timestamp);
    }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

