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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraSubsystem extends SubsystemBase {
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new ExampleSubsystem. */
  public CameraSubsystem() {}
   

  public Pose2d getCameraPositionFromAprilTag() {
    // read camera position realtive to april tag location from limelight
    // NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
    double[] cameraPosition = limelight.getEntry("camtran").getDoubleArray(new double[]{});
    System.out.println("April Tag Location: " + cameraPosition);
    if(cameraPosition.equals(new double[] {0.0d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d})){
      // no April Tag detected
      System.out.println("WARNING: No april tag detected");
      return (Pose2d) null;
    }
    else 
    {
      // unpack cameraPosition array into a Pose
      return new Pose2d(cameraPosition[0], cameraPosition[1], new Rotation2d(cameraPosition[4]));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
