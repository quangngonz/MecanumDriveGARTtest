// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.SUBSYSTEM.*;

public class DriveBase extends SubsystemBase {
  public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RIGHTMOTOR1_ID);
  public WPI_TalonSRX rearRightMotor  = new WPI_TalonSRX(RIGHTMOTOR2_ID);
  public WPI_TalonSRX frontLeftMotor  = new WPI_TalonSRX(LEFTMOTOR1_ID);
  public WPI_TalonSRX rearLeftMotor   = new WPI_TalonSRX(LEFTMOTOR2_ID);

  public MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  /** Creates a new DriveBase. */
  public DriveBase() {
  }

  /** 
  private double calculateAngle(double x, double y) {
    double angle = Math.atan2(y, x);
    if (x>=0 && y>=0){
      ;
    }
    if (x>=0 && y<0) {
    //quadrant 4
    angle=360+angle;
    }
    if (x<0 && y<0) {
    //quadrant 3
    angle=180-angle;
    }
    if (x<0 && y>0) {
    //quadrant 2
    angle=180-angle;
    }
    return angle;
  }  */

  @Override
  public void periodic() {
    double LX = RobotContainer.xbox.getLeftX();
    double LY = RobotContainer.xbox.getLeftY();
    double ZR = RobotContainer.xbox.getRightX();

    // double angle = calculateAngle(LX, LY);
    // mecanumDrive.driveCartesian(LX, LY, ZR, angle);

    if (RobotContainer.xbox.getLeftTriggerAxis() != 0 | RobotContainer.xbox.getLeftBumperPressed()) {
      mecanumDrive.driveCartesian(LX*0.8, LY*0.8, ZR);
    } else {
      mecanumDrive.driveCartesian(LX*0.6, LY*0.6, ZR);
    }
    // This method will be called once per scheduler run
  }
}
