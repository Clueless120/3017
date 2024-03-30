// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private TalonFX rightPivot, leftPivot;
  private PIDController pivotController;
  private CANcoder canCoder;
  public Pivot() {
    rightPivot = new TalonFX(PivotConstants.kRightID);
    leftPivot = new TalonFX(PivotConstants.kLeftID);
    rightPivot.setInverted(true);
    leftPivot.setControl(new Follower(rightPivot.getDeviceID(), true));
    
    pivotController = new PIDController(3.4, 0, 0);
    canCoder = new CANcoder(PivotConstants.kpivotCanCoderID);
    pivotController.setTolerance(PivotConstants.kTolarance);
    
  }
  public void goTo(double setPoint){
    rightPivot.set(pivotController.calculate(getPosition(), setPoint));
  }
  public double getPosition(){
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }
  public void runPivot(double value){
    rightPivot.set(value);
  }
  public void stop(){
    rightPivot.set(0);
  }
  public boolean atSetpoint(){
    return pivotController.atSetpoint();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CANCoder", getPosition());
  }
}
