package frc.robot.Subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain {
   private final WPI_TalonSRX leftDriveTalon;
   private final WPI_TalonSRX rightDriveTalon;  
   private AHRS navx = new AHRS(SPI.Port.kMXP);

  public DriveTrain () {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    leftDriveTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveTalon.setNeutralMode(NeutralMode.Brake);

    leftDriveTalon.setInverted(true);
    rightDriveTalon.setInverted(false);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);
    
    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    }

    public void TankDrive(double leftSpeed, double rightSpeed) {
      rightDriveTalon.set(rightSpeed);
      leftDriveTalon.set(leftSpeed);
    }

    public void resetEncoders() {
      leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
      rightDriveTalon.setSelectedSensorPosition(0, 0, 10);
    }
  
    public double getTicks() {
      return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
    }
  
    // Converts ticks to meters from the encoders
    public double TicksToMeters() {
      return (getTicks() * (Units.inchesToMeters(6) * Math.PI) / 4096);
    }
  
    public double getAngle() {
      return navx.getAngle();
    }

    public double getHeading(){
      // Negative sign means the angle is measured counter clockwise from a reference point
      return -navx.getRotation2d().getDegrees();
    }

    public void resetNavx() {
      navx.reset();
    }
}
