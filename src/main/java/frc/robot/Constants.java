// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class CAN {

    //Starts at 2 because Rio and PDH are 0 and 1, respectively
    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kBackRight = 3;
    public static final int kBackLeft = 4;
    public static final int kPigeon = 6;


  }

  public static class DriveConstants {

    //ENCODERS YAY FUN 
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kFalconToMeters = (1.0/2048) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71) ; //10.71:1 gearbox

    public static final double DRIVETRAIN_kS = 0.13305;
    public static final double DRIVETRAIN_kV = 2.2876;
    public static final double DRIVETRAIN_kA = 0.31596;

    public static final double[] DRIVETRAIN_CHARACTERIZATION = {DRIVETRAIN_kS, DRIVETRAIN_kV, DRIVETRAIN_kA};


  }
}
