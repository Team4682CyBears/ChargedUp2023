// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: OnboardInputInterfaces.java
// Intent: Forms a class that grants access to onboard inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.common.*;
import java.util.*; 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class OnboardInputInterfaces implements Sendable
{
    private AHRS navigationSensor = null;
    private double yawOffset = 0.0;

    /**
     * The constructor to build this 'onboard input' conduit
     */
    public OnboardInputInterfaces()
    {
        if(InstalledHardware.navxInstalled)
        {
            navigationSensor = new AHRS(SPI.Port.kMXP);
        }
    }

    /**
     * Initalize the onboard interfaces
     * @param yawOffsetFromFieldLongAxis - Positive values are clockwise offset from long axis of FRC field, negtave are counterclockwise.
     * Parameter intended to allow the robot to reference its position from a known axis on the field.  North (heading 0.0) is assumed to be
     * on the side of the starting side of the field (red or blue). 
     */
    public void initialize(double yawOffsetFromFieldLongAxis)
    {
        if(navigationSensor != null)
        {
            navigationSensor.zeroYaw();
        }
        yawOffset = yawOffsetFromFieldLongAxis;
    }

    /**
     * A method to obtain the current match target color
     * @return - the target color of the current match
     */
    public Color getCurrentMatchTargetColor()
    {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.Blue : Color.Red;
    }

    /**
     * A method to obtain the balls within the field of vision for the limelight
     * @return - a vector of all of the balls in the field of vision
     */
    public Vector<CoordinateSystemBall> getBallsInFieldOfVision()
    {
        Vector<CoordinateSystemBall> balls = new Vector<CoordinateSystemBall>();

        // TODO - OWEN fill in the code here to locate all balls within the field of vision of limelight

        // This is just an example below.  The jist is would be you need to:
        // 1. determine circular objects with the limelight
        int countOfBallsFound = 1;
        for(int inx = 0; inx < countOfBallsFound; ++inx)
        {
            // 2. determine each balls centroid in reference to a 0,0 coordinate system in the center of the field of vision
            double nextXCentroid = -100.5; // pixels? (-x implies left of center)
            double nextYCentroid = 200.5; // pixels? (+y implies above center)
            // 3. determine each balls general diameter
            double nextDiameter = 1.0; // pixels?
            // 4. determine each balls color
            Color nextColor = Color.Red;
            // 5. then create an CoordinateSystemBall object and insert it into the collection
            CoordinateSystemBall nextBallInField = new CoordinateSystemBall(nextColor, nextDiameter, nextXCentroid, nextYCentroid);
            // insert the newly created ball into the vector/array
            balls.add(nextBallInField);
        }

        // make sure to sort before returning
        Collections.sort(balls);
        return balls;
    }

    /**
     * A method to get the pitch angle of the robot
     * 
     * see: https://pdocs.kauailabs.com/navx-mxp/guidance/terminology/ for more information on what is in this method
     * Note: this code assumes the RoboRio is laying horizontal with NavX attached directly
     * 
     * @return - a double value associated with the magnitude of the pitch (-180.0 to 180.0) for the robot.  Positive values imply nose up attitude.
     */
    public double getPitchAngle()
    {
        double rtnVal = 0.0;
        if(navigationSensor != null)
        {
            switch(Constants.roboRioOrientation)
            {
                // navx - X positive direction, implies pitch comes from negative roll
                case RelayForward:
                    rtnVal = navigationSensor.getRoll() * -1.0;
                    break;
                // navx - X negative direction, implies pitch comes from positive roll
                case UsbForward:
                    rtnVal = navigationSensor.getRoll();
                    break;
                // navx - Y negative direction, implies pitch comes from negative pitch
                case DioForward:
                    rtnVal = navigationSensor.get() * -1.0;
                    break;
                // navx - Y positive direction, implies pitch comes from positive pitch
                case PwmForward:
                    rtnVal = navigationSensor.getPitch();
                    break;
            }

        }
        return rtnVal;
    }

    /**
     * A method to get the roll angle of the robot
     * 
     * see: https://pdocs.kauailabs.com/navx-mxp/guidance/terminology/ for more information on what is in this method
     * Note: this code assumes the RoboRio is laying horizontal with NavX attached directly
     * 
     * @return - a double value associated with the magnitude of the roll (-180.0 to 180.0) for the robot.  Positive values imply left hand roll (left lower than right).
     */
    public double getRollAngle()
    {
        double rtnVal = 0.0;
        if(navigationSensor != null)
        {
            switch(Constants.roboRioOrientation)
            {
                // navx - X positive direction, implies roll comes from negative pitch
                case RelayForward:
                    rtnVal = navigationSensor.getPitch() * -1.0;
                    break;
                // navx - X negative direction, implies roll comes from positive pitch
                case UsbForward:
                    rtnVal = navigationSensor.getPitch();
                    break;
                // navx - Y negative direction, implies roll comes from negative roll
                case DioForward:
                    rtnVal = navigationSensor.getRoll() * -1.0;
                    break;
                // navx - Y positive direction, implies roll comes from positive roll
                case PwmForward:
                    rtnVal = navigationSensor.getRoll();
                    break;
            }

        }
        return rtnVal;
    }

    /**
     * A method to get the yaw angle of the robot - this is the 'spin angle'
     * 
     * see: https://pdocs.kauailabs.com/navx-mxp/guidance/yaw-drift/ for info about yaw drift
     * Note: this code assumes the RoboRio is laying horizontal with NavX attached directly
     * 
     * @return - a double value associated with the magnitude of the yaw (-180.0 to 180.0) for the robot.  Positive values are clockwise, negative are counter-clockwise.
     */
    public double getYawAngle()
    {
        double rtnVal = 0.0;
        if(navigationSensor != null)
        {
            rtnVal = navigationSensor.getYaw() + yawOffset;
            if(rtnVal > 180.0)
            {
                rtnVal = -180.0 + (rtnVal - 180.0);
            }
            else if(rtnVal < -180.0)
            {
                rtnVal = 180.0 - (rtnVal + 180.0);
            }
        }
        return rtnVal;
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("NavxPitchAngle", this::getPitchAngle, null);
        builder.addDoubleProperty("NavxRollAngle", this::getRollAngle, null);
        builder.addDoubleProperty("NavxYawAngle", this::getYawAngle, null);
        builder.addDoubleProperty("NavxYawAngleOffset", this::getYawAngleOffset, this::setYawAngleOffset);
    }

    private double getYawAngleOffset()
    {
        return this.yawOffset;
    }

    private void setYawAngleOffset(double value)
    {
        this.yawOffset = value;
    }
}
