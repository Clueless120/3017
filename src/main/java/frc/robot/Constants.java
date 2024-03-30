// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.attribute.FileOwnerAttributeView;

/** Add your docs here. */
public class Constants {
    public final class IndexerConstants {
    public static final int  kID = 14;
    public static final double kShootingPower = 0.75;
    }
    public final class  PivotConstants {
    public static final int kRightID = 13;
    public static final int kLeftID = 10;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kSourcePosition = 0.154296875;
    public static final double kShootingPosition = 0.111572265625; 
    public static final int kpivotCanCoderID = 55;  
    public static final double kTolarance = 0.019;    
}
    public static class  ShooterConstants {
    public static final int KbackLeftMotorID = 15 ; 
    public static final int kbackRightMotorID = 9;
    public static final int kfrontLeftMotorID = 12;
    public static final int kfrontRightMotorID = 11;
    public static final double kShootingPower = 0.85;
    public static final double kIntakingPower = -0.5;

    
    }
}
