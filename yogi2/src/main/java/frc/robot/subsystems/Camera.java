// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: Camera.java
// Intent: Forms a subsystem that reads and process vision.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

//TODO make a selector to choose ball color to target

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase
{
    /*
      The PhotonCamera class has two constructors: one that takes a NetworkTable 
      and another that takes in the name of the network table that PhotonVision is
      broadcasting information over. For ease of use, it is recommended to use the latter.
      The name of the NetworkTable (for the string constructor) should be the same as the 
      camera’s nickname (from the PhotonVision UI).
    */
    PhotonCamera camera = new PhotonCamera("gloworm");

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    DriveTrain driveTrain;

    /** Creates a new Camera. */
    public Camera()
    {
    }

    @Override
    public void periodic()
    {
      // This method will be called once per scheduler run
    }

    public void targetObject(double forwardSpeed, double rotationSpeed)
    {
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets())
      {
          // Calculate angular turn power
          // -1.0 required to ensure positive PID controller effort _increases_ yaw
          rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      }
      else
      {
          // If we have no targets, stay still.
          rotationSpeed = 0;          
      } 

      // Use our forward/turn speeds to control the drivetrain
      driveTrain.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
    public boolean hasTargets()
    {
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();
      // Check if the latest result has any targets.
      return result.hasTargets();
    }

    // Drivermode is a display option for comera feed.
    public void setDriverModeOn()
    {
      // Set driver mode to on.
      camera.setDriverMode(true);
    }

    public void setDriverModeOff()
    {
      // Set driver mode to off.
      camera.setDriverMode(false);
    }

    public void ledsOn()
    {
      camera.setLED(VisionLEDMode.kOn);
    }

    public void ledsOff()
    {
      camera.setLED(VisionLEDMode.kOff);
    }   
}
