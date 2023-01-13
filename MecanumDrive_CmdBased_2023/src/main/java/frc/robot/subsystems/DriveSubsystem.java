// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // drive motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax (Constants.DriveConstants.kfrontLeftMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax   (Constants.DriveConstants.krearLeftMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax (Constants.DriveConstants.kfrontRightMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax  (Constants.DriveConstants.krearRightMotorDeviceID, MotorType.kBrushless);

  // motor encoders * guide only intanciated front left and front right*
  private final RelativeEncoder frontLeftEncoder =  frontLeftMotor.getEncoder();
  //private final RelativeEncoder rearLeftEncoder  =  rearLeftMotor.getEncoder();
  private final RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  //private final RelativeEncoder rearRightEncoder  = rearRightMotor.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // set inverted the right drive motors
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
    // set inverted the right drive encoder
   // frontRightEncoder.setInverted(true);

    // reset drive encoders
    frontRightEncoder.setPosition(0);
    frontLeftEncoder.setPosition(0);
  }

  public void setMotors(double leftSpeed, double rightSpeed){
    // apply deadBand
    if(Math.abs(leftSpeed) <= Constants.OperatorConstants.kDeadband) leftSpeed =0.0;
    if(Math.abs(rightSpeed) <= Constants.OperatorConstants.kDeadband) rightSpeed =0.0;

    //apply power to motors
    frontLeftMotor.set(leftSpeed);
    rearLeftMotor.set(leftSpeed);
    frontRightMotor.set(rightSpeed);
    rearRightMotor.set(rightSpeed);
  }

  public void setMotors(double frontLeftSpeed, double rearLeftSpeed, double frontRightSpeed, double rearRightSpeed){
    frontLeftMotor.set(frontLeftSpeed);
    rearLeftMotor.set(rearLeftSpeed);
    frontRightMotor.set(frontRightSpeed);
    rearRightMotor.set(rearRightSpeed);
  }

// method gets the distance bot has traveled in feet. Value is average of frontLeftEncoder and frontRightEncoder readings
public double getDriveEncoderFt(){
  return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) /2 * Constants.DriveConstants.kDriveTick2Feet;
 }
 
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //display the distance the robot traveled on the dashboard
    SmartDashboard.putNumber("Drive Encoder vaule in Ft", getDriveEncoderFt());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // cool easter egg and test
  }
}
