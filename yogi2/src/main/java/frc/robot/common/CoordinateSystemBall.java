// ************************************************************
// Bishop Blanchet Robotics
// Historic home of the 'BraveBots'
// FRC - Rapid React - 2022
// File: CoordinateSystemBall.java
// Intent: Forms model for a ball in limelight that maps to a coordinate system where 0,0 is in the center of the field of vision.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.lang.*;

/**
 * A ball that is discovered within the limelight field of vision where the coordinate
 * system is 0,0 at the center of the field of vision, representing straight ahead
 * 
 * Expectations
 * +x value implies the ball centroid is right of the center of field of vision
 * -x value implies the ball centroid is left of the center of field of vision
 * +y value implies the ball centroid is above the center of field of vision
 * -y value implies the ball centroid is below the center of field of vision
 */
public class CoordinateSystemBall implements Comparable
{
    private Color color = Color.Blue;
    private double diameter = 0.0;
    private double xCentriod = 0.0;
    private double yCentriod = 0.0;

    /**
     * Constructor for a ball that is discovered within the limelight field of vision where the coordinate
     * system center of 0,0 is in the middle top-to-bottom and left-to-right
     * @param ballColor - The color of the ball at hand
     * @param ballDiameter - The diameter of the ball at hand
     * @param xBallCentriod - The x axis centriod point of the ball within the coordinate system  
     * @param yBallCentroid - The y axis centriod point of the ball within the coordinate system
     */
    public CoordinateSystemBall(
        Color ballColor,
        double ballDiameter,
        double xBallCentriod,
        double yBallCentroid)
    {
        color = ballColor;
        diameter = ballDiameter;
        xCentriod = xBallCentriod;
        yCentriod = yBallCentroid;
    }

    /**
     * Get the color of the ball instance
     * @return the color of the ball
     */
    public Color getColor() { return color; }

    /**
     * Get the diameter of the ball within the coordinate system
     * note: larger the ball the closer the ball is in the field of vision
     * @return the diameter of the ball within the coordinate system
     */
    public double getBallDiameter() { return diameter; }

    /**
     * Get the X coordinate (side-to-side) of the centrioid of the ball
     * @return the X coordinate of the centrioid of the ball
     */
    public double getBallCentroidX() { return xCentriod; }

    /**
     * Get the Y coordinate (up/down) of the centrioid of the ball
     * @return the Y coordinate of the centrioid of the ball
     */
    public double getBallCentroidY() { return yCentriod; }

    @Override
    public int compareTo(Object ball)
    {
        CoordinateSystemBall rightBall = (CoordinateSystemBall)ball;
        return this.diameter == rightBall.diameter ? 0 : (this.diameter > rightBall.diameter ? 1 : -1);
    }
}
