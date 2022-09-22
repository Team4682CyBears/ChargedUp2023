// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import frc.robot.commands.*;
import frc.robot.common.Types.BeamBreakState;
import frc.robot.subsystems.*;

public class ManualInputInterfaces
{
  private BeamBreakSensor bbSensor = new BeamBreakSensor(Constants.beamBreakSensorPort);

  /**
   * A method to get the beam break state
   * @return - an enum value associated with the beam break state
   */
  public BeamBreakState getBeamBreakState()
  {
    return bbSensor.getState();
  }

}