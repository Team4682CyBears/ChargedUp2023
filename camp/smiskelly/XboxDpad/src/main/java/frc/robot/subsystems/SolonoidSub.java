// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: SolonoidSub.java
// Intent: Wrapper class standard stub for robot in FRC challange.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

import frc.robot.*;

  
public class SolonoidSub extends SubsystemBase {

  private Compressor compressor = new Compressor(
    Constants.PneumaticsControlModuleNumber,
    Constants.TestPneumaticsControlModuleType);

  private DoubleSolenoid solenoid = new DoubleSolenoid(
      Constants.PneumaticsControlModuleNumber,
      Constants.TestPneumaticsControlModuleType,
      Constants.PneumaticsControlModuleForwardChannel,
      Constants.PneumaticsControlModuleReverseChannel);

  /** Creates a new ExampleSubsystem. */
  public SolonoidSub() {
    this.solenoid.set(DoubleSolenoid.Value.kOff);
  }

  /**
   * Method to move the cylindar into a deployed (upward) position
   */
  public void deployPosition(){
      this.solenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Method to move the cylindar into a retracted (downward) position
   */
  public void retractPosition(){
      this.solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
