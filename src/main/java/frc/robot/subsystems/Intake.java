// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.SUBSYSTEM.*;

public class Intake extends SubsystemBase {
  public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_ID);
  private int onOff = 0;

  /** Creates a new Intake. */
  public Intake() {}

  public void intake(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    if (RobotContainer.xbox.getRightTriggerAxis() != 0 | RobotContainer.xbox.getRightBumperPressed()) {
      onOff += 1;
      onOff = onOff % 2;
    }
    if (onOff == 1) {
      intake(0.5);
    } else {
      intake(0);
    }
    // This method will be called once per scheduler run
  }
}
