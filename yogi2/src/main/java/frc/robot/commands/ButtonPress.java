// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ButtonPress.java
// Intent: Forms a manual command to print the button number.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import java.util.ArrayDeque;
import java.util.Iterator;

public class ButtonPress extends CommandBase implements Sendable
{
  private static final int maxPreviousButtonCount = 10;
  private static final double roundPrecision = 100.0;
  private static ArrayDeque<String> previousButtons = new ArrayDeque<String>(ButtonPress.maxPreviousButtonCount);
  private static Timer timer = new Timer();
  private static boolean timerStarted = false;

  private String inputDevice = "";
  private String inputAction = "";
  private double initTime = 0.0;
  private double executeTime = 0.0;
  private double finalTime = 0.0;

  /**
   * The constructor 
   */
  public ButtonPress(
    String inputDeviceDescription,
    String inputActionDescription)
  {
      inputDevice = inputDeviceDescription;
      inputAction = inputActionDescription;
      if(timerStarted == false)
      {
        timerStarted = true;
        timer.reset();
        timer.start();
      }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    initTime = timer.get();
    executeTime = 0.0;
    finalTime = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    executeTime = timer.get();
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.addStringProperty("PreviousButtonPress0", ButtonPress::getButtonDescription0, null);
    builder.addStringProperty("PreviousButtonPress1", ButtonPress::getButtonDescription1, null);
    builder.addStringProperty("PreviousButtonPress2", ButtonPress::getButtonDescription2, null);
    builder.addStringProperty("PreviousButtonPress3", ButtonPress::getButtonDescription3, null);
    builder.addStringProperty("PreviousButtonPress4", ButtonPress::getButtonDescription4, null);
    builder.addStringProperty("PreviousButtonPress5", ButtonPress::getButtonDescription5, null);
    builder.addStringProperty("PreviousButtonPress6", ButtonPress::getButtonDescription6, null);
    builder.addStringProperty("PreviousButtonPress7", ButtonPress::getButtonDescription7, null);
    builder.addStringProperty("PreviousButtonPress8", ButtonPress::getButtonDescription8, null);
    builder.addStringProperty("PreviousButtonPress9", ButtonPress::getButtonDescription9, null);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    finalTime = timer.get();
    String buttonPressDescription = this.toString();
    System.out.println(buttonPressDescription);
    previousButtons.addFirst(buttonPressDescription);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }

  @Override
  public String toString()
  {
    return inputDevice + ":" +
      inputAction + ":" +
      Math.round(initTime*ButtonPress.roundPrecision)/ButtonPress.roundPrecision + ":" +
      Math.round(executeTime*ButtonPress.roundPrecision)/ButtonPress.roundPrecision + ":" +
      Math.round(finalTime*ButtonPress.roundPrecision)/ButtonPress.roundPrecision;
  }


  private static String getButtonDescription0()
  {
    return ButtonPress.getButtonDescription(0);
  }

  private static String getButtonDescription1()
  {
    return ButtonPress.getButtonDescription(1);
  }

  private static String getButtonDescription2()
  {
    return ButtonPress.getButtonDescription(2);
  }

  private static String getButtonDescription3()
  {
    return ButtonPress.getButtonDescription(3);
  }

  private static String getButtonDescription4()
  {
    return ButtonPress.getButtonDescription(4);
  }

  private static String getButtonDescription5()
  {
    return ButtonPress.getButtonDescription(5);
  }

  private static String getButtonDescription6()
  {
    return ButtonPress.getButtonDescription(6);
  }

  private static String getButtonDescription7()
  {
    return ButtonPress.getButtonDescription(7);
  }

  private static String getButtonDescription8()
  {
    return ButtonPress.getButtonDescription(8);
  }

  private static String getButtonDescription9()
  {
    return ButtonPress.getButtonDescription(9);
  }

  private static String getButtonDescription(int index)
  {
    // keep the button descriptions pruned to the right size
    for(int jnx = previousButtons.size() - ButtonPress.maxPreviousButtonCount; jnx > 0; --jnx)
    {
      previousButtons.removeLast();
    }

    String rtnVal = "";
    if(previousButtons.size() > index)
    {
      String[] values = previousButtons.toArray(new String[0]);
      rtnVal = values[index];
    }
    return rtnVal;
  }
}
