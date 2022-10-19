// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  

  //JOYSTICK
  public Joystick driveStick;

  public double power;
  public double rotation;

  //CREATE MOTOR CONTROLLER OBJECTS AND GROUP THEM
  //RIGHT SIDE
  public MotorController m_rearRightController = new WPI_VictorSPX(Constants.REAR_RIGHT);
  public MotorController m_frontRightController = new WPI_VictorSPX(Constants.FRONT_RIGHT);

  public MotorControllerGroup m_right; 

  //LEFT SIDE
  public MotorController m_rearleftController = new WPI_VictorSPX(Constants.REAR_LEFT);
  public MotorController m_frontleftController = new WPI_VictorSPX(Constants.FRONT_LEFT);

  public MotorControllerGroup m_left;

  //DIFFERENTIALDRIVE OBJECT
  public DifferentialDrive drive;

  public DriveSubsystem() {
    m_left = new MotorControllerGroup(m_frontleftController, m_rearleftController);
    m_right = new MotorControllerGroup(m_rearRightController, m_frontRightController);
    m_right.setInverted(true);
    driveStick = new Joystick(Constants.driveJoystick);
    drive = new DifferentialDrive(m_left, m_right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveStraight(){
    drive.tankDrive(0.6, 0.6);
  }

  public void arcadeDrive(Double power, Double rotation){
    drive.arcadeDrive(power, rotation);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

