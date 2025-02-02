// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  public DriveTrain dt;
  public Joystick joy;

  public TankDrive(DriveTrain dt, Joystick joy) {
    this.dt = dt;
    this.joy = joy;
    addRequirements(dt);
  }

  private void addRequirements(DriveTrain dt) {
}

@Override
  public void initialize() {
    dt.TankDrive(0,0);
  }

  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(Constants.DriveTrainPorts.LeftJoystickPort);
    double rightPowerRaw = joy.getRawAxis(Constants.DriveTrainPorts.RightJoystickPort);
    dt.TankDrive(leftPowerRaw*0.3, rightPowerRaw*0.3);
  }

  @Override
  public void end(boolean interrupted) {
    dt.TankDrive(0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}