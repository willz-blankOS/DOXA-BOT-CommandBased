// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  //GAINS
  public double kP = 1;
  public double kI = 1;
  public double kD = 1;

  //PID CONTROLLER
  public PIDController pid = new PIDController(kP, kI, kD);

  //SIMPLE MOTOR FEED
  SimpleMotorFeedforward feedforward;

  //JOYSTICK
  public Joystick driveStick;

  //NAV-X
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

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
    feedforward = new SimpleMotorFeedforward(kP, kI);
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

  public void arcadeDrive(Double power, Double rotation){
    if(power > 0.15 || power < -0.15){
      m_left.setVoltage(feedforward.calculate(power));
      m_right.setVoltage(feedforward.calculate(power));
    }
    if(rotation > 0.15 || rotation < -0.15){
      m_left.setVoltage(feedforward.calculate(rotation));
      m_right.setVoltage(feedforward.calculate(-rotation));
    }
  }

  public void tankDrive(Double leftVelocity, Double rightVelocity) {
    m_left.setVoltage(feedforward.calculate(leftVelocity));
    m_right.setVoltage(feedforward.calculate(rightVelocity));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}