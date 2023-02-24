// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: NavxSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.EulerAngle;
import frc.robot.common.VectorUtils;
import java.util.ArrayList;
import java.lang.Math;

public class NavxSubsystem extends SubsystemBase {

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  public final AHRS swerveNavx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // store yaw/pitch history
  private static int LevelListMaxSize = 20;
  private ArrayList<Float> RecentRolls = new ArrayList<Float>();
  private ArrayList<Float> RecentPitches = new ArrayList<Float>();

  /** Creates a new NavX. */
  public NavxSubsystem() {}

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    if(swerveNavx.isCalibrating()){
      // From the NavX Docs: This method has no effect if the sensor is currently calibrating
      System.out.println("WARNING: Gyro is calibrating. Zeroing gyro has no effect while it is calibrating.");
    }
    swerveNavx.zeroYaw();
  }

  private void storeRoll(){
    this.RecentRolls.add(this.swerveNavx.getRoll());
    while(this.RecentRolls.size() > LevelListMaxSize)
    {
      RecentRolls.remove(0);
    }
  }
  private void storePitch(){
    RecentPitches.add(swerveNavx.getPitch());
    while(RecentPitches.size() > LevelListMaxSize)
    {
      RecentPitches.remove(0);
    }
  }

  /**
   * Determines if the navx is level.  
   * @return true if level, false otherwise
   */
  public boolean isLevel() {
   return areAllLevel(RecentPitches) && areAllLevel(RecentRolls);
  }

  /**
   * A method to get the quaternion representation of the navx pose. 
   * @return quaternion
   */
  public Quaternion getQuaterion() {
    Quaternion q = new Quaternion(swerveNavx.getQuaternionW(), swerveNavx.getQuaternionX(), swerveNavx.getQuaternionY(),swerveNavx.getQuaternionZ());
    return (q);
  }

  /**
   * returns navx euler angle (pitch, roll, yaw) in degrees
   * @return EulerAngle
   */
  public EulerAngle getEulerAngle(){
    return new EulerAngle(
      swerveNavx.getPitch(), 
      swerveNavx.getRoll(), 
      swerveNavx.getYaw());
  }

  private boolean areAllLevel(ArrayList<Float> recentAngles){
    boolean levelChecker = true;
    for(int i = 0; i < LevelListMaxSize; i++){
      if (Math.abs(recentAngles.get(i))>=Constants.navxTolDegrees){
        levelChecker = false;
      }
    }
    System.out.println("Is Level? " + levelChecker);
    return levelChecker;
  }

  public ArrayList<Float> getRecentPitches(){
    return RecentPitches;
  }

  public ArrayList<Float> getRecentRolls(){
    return RecentRolls;
  }

  /**
   * a method to print relevant state of the navx
   */
  public void printState(){
    System.out.println("**** NavX State ****");
    System.out.println("Quaternion ------>" + this.getQuaterion());
    System.out.println("Roll, Pitch, Yaw ------>" + this.getEulerAngle());
    System.out.println("Is the robot level? -------->" + this.isLevel());
    System.out.println("SteepestAscent --->" + VectorUtils.getAngleOfSteepestAscent(getEulerAngle()));
  }

  /**
   * Obtains the current gyroscope's rotation about the Z axis when looking down at the robot where positive is measured in the 
   * counter-clockwise direction.
   * 
   * Notes: 
   * According to https://pdocs.kauailabs.com/navx-mxp/wp-content/uploads/2020/09/navx2-mxp_robotics_navigation_sensor_user_guide-8.pdf
   * NavX .getYaw() is -180 to 180 where positive is clockwise looking at the sensor from above
   * According to https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
   * positive rotations are measured in the counter-clockwise looking at the robot from above
   * these two pieces of information imply the yaw should be inverted (or multiplied by -1.0)
   * @return A Rotation2d that describes the current orentation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {

    // TODO - we need to have someone determine if our existing (NavX v1) setup will make use of the
    // 'getFusedHeading' or if it uses the 'getYaw' method (e.g., if isMagnetometerCalibrated() or not)
    // to run this test all we need is for someone to comment out the System.out.println lines of code below

    if (swerveNavx.isMagnetometerCalibrated()) {

      // TODO - test this!!
      // System.out.println("getGyroscopeRotation() using: swerveNavx.getFusedHeading()");

      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(swerveNavx.getFusedHeading());
    }

    // TODO - test this!!
    // System.out.println("getGyroscopeRotation() using: swerveNavx.getYaw()");

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - swerveNavx.getYaw());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    storePitch();
    storeRoll();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
