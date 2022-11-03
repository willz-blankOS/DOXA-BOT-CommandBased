// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive; /** DRIVE SUBSYSTEM */
  private final LimelightSubsystem limelight; /** LIMELIGHT SUBSYSTEM !! NOT BEING USED YET !! */

  // PID VALUES / GAINS
  private final double kP = 0.35; /** PERIPHERAL */
  private final double kI = 0.1; /** INTEGRAL */

  // DOUBLE VARIABLES TO STORE LIMELIGHT VALUES
  private double x; /** HORIZONTAL OFFSET */
  private double y; /** VERTICAL OFFSET */
  private double area; /** TARGET AREA */
  private boolean isTarget; /** CHECK IF THERE IS A TARGET */

  // NETWORK TABLE OBJECTS TO GET LIMELIGHT VALUES FROM SHUFFLEBOARD
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDrive(DriveSubsystem drive, LimelightSubsystem limelight) {
    this.drive = drive;
    this.limelight = limelight; /** !! NOT BEING USED YET !! */
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive.setSafetyEnabled(true);
    drive.m_left.setInverted(false); 
    drive.m_right.setInverted(true); /** INVERT RIGHT SIDE */
    
    // TO OBTAIN LIMELIGHT VALUES
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    isTarget = tv.getBoolean(false);

    // DRIVE
    if(!drive.driveStick.getRawButton(8) || !isTarget){ /** IF ROBOT IS NOT MEANT TO SNAP TO TARGET */
      drive.arcadeDrive(drive.driveStick.getRawAxis(1) * 10, drive.driveStick.getRawAxis(2) * 10);
    }else if(drive.driveStick.getRawButton(8)){ /** IF ROBOT IS MEANT TO SNAP TO TARGET */
      if(isTarget){
        if(drive.driveStick.getRawAxis(1) < 0.1 && drive.driveStick.getRawAxis(1) > -0.1){ /** CHECK IF THERE IS NOT USER INPUT */
          drive.m_left.setVoltage( kP * x);
          drive.m_right.setVoltage( kP * -x);
        }else{ /** IF THERE IS USER INPUT */
          drive.m_left.setVoltage(drive.driveStick.getRawAxis(1) * 4 + (kP * x));
          drive.m_right.setVoltage(drive.driveStick.getRawAxis(1) * 4 + (kP * -x));
        }  
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
