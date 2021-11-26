// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Vision extends PIDSubsystem {
  private NetworkTable gripTable;
  private Target mainTarget = new Target();

  public Vision() {
    super(new PIDController(0.002, 0.0, 0.0));

    gripTable = NetworkTableInstance.getDefault().getTable("GRIP").getSubTable("targetData");

    setSetpoint(0);
    getController().setTolerance(Constants.POSITION_TOLERANCE, Constants.VELOCITY_TOLERANCE);
  }

  public void updateShuffleboard() {
    SmartDashboard.putNumber("Target Center X", mainTarget.getCenterX());
    SmartDashboard.putNumber("Target Center Y", mainTarget.getCenterY());
    SmartDashboard.putNumber("Target Area", mainTarget.getArea());
    SmartDashboard.putBoolean("Target Exists", mainTarget.exists());
  }

  public Target getTarget(){
    return mainTarget;
  }

  @Override
  protected double getMeasurement() {
    return mainTarget.getCenterX();
  }

  @Override
  protected void useOutput(double output, double setpoint) {}

  @Override
  public void periodic() {
    if(gripTable.getEntry("centerX").getDoubleArray(new double[1]).length != 0){
      mainTarget.updateValues(
        gripTable.getEntry("centerX").getDoubleArray(new double[1])[0], 
        gripTable.getEntry("centerY").getDoubleArray(new double[1])[0], 
        gripTable.getEntry("area").getDoubleArray(new double[1])[0]);
    } else if(mainTarget.exists()){
      mainTarget.setExistanceToFalse();
    }
  }

  public class Target {

    private double centerX;
    private double centerY;
    private double area;

    private boolean exists = false;

    public Target(double centerX, double centerY, double area) {
      this.centerX = centerX;
      this.centerY = centerY;
      this.area = area;
      exists = true;
    }

    public Target() {
      exists = false;
      this.centerX = 0;
      this.centerY = 0;
      this.area = 0;
    }

    public double getCenterX() {
      return centerX;
    }

    public double getCenterY() {
      return centerY;
    }
    
    public double getArea() {
      return area;
    }

    public boolean exists() {
      return exists;
    }

    public void updateValues(double centerX, double centerY, double area) {
      this.centerX = centerX - 320;
      this.centerY = centerY;
      this.area = area;
      exists = true;
    }

    public void setExistanceToFalse() {
      updateValues(0, 0, 0);
      exists = false;
    }
  }
}
