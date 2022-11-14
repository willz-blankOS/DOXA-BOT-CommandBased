// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FullRight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;

  private double Error;
  private double target;
  private double kP = 0.45;
  private double kI = 0.1;
  private double kD = 1;
  private double kV;
  private double kS;
  
  private SimpleWidget getHeading;
  private SimpleWidget getTargetHeading;

  private double timer = 0;

  // SIMPLE MOTOR FEED
  private SimpleMotorFeedforward forwardFeed = new SimpleMotorFeedforward(kS, kV);

  // PID CONTROLLER
  private PIDController pid = new PIDController(kP, kI, kD);

  /**
   * Creates a new FullRight.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FullRight(DriveSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 /*   
    if(drive.gyro.getAngle() + 90 > 360){
      target = 360 - drive.gyro.getAngle();
    }else{
      target = drive.gyro.getAngle();
    }
    
    Error = drive.gyro.getAngle() - target;
  */
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Error = drive.gyro.getAngle() - target;
    drive.arcadeDrive(0.0, kP * Error);
  } 


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timer > 3){
      return true;
    }else{
      return false;
    }
  }
}