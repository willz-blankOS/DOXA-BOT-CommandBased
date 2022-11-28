// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;
  private Command auto;
  private RobotContainer m_robotContainer;
  
  private CameraServer limelightCameraServer;

  private AnalogGyro Gyro = new AnalogGyro(0);

  // SHUFFLEBOARD ELEMENT TO SELECT AUTO MODE
  private SendableChooser<String> m_chooser = new SendableChooser<String>();
  private SendableChooser<String> m_teleopchooser = new SendableChooser<String>();
  private String autoMode; /** String to get Auto mode */
  private String teleopMode;

  private double x;
  private double y;
  private double area;
  
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    
    SmartDashboard.getNumber("Limelight X", x);
    SmartDashboard.getNumber("Limelight Y", y);
    SmartDashboard.getNumber("Limelight A", area);


    //ADDING AUTO OPTIONS
    m_chooser.setDefaultOption("STRAIGHT_LINE_Auto", "STRAIGHT_LINE_AUTO");
    m_chooser.addOption("90_TURN_AUTO", "90_TURN_AUTO");
    m_chooser.addOption("KICK_THE_BOT_AUTO", "KICK_THE_BOT_AUTO");
    SmartDashboard.putData("AUTO MODES", m_chooser);

    m_teleopchooser.setDefaultOption("DEFAULT DRIVE", "DEFAULT DRIVE");
    m_teleopchooser.addOption("CURVE DRIVE", "CURVE DRIVE");
    SmartDashboard.putData("TELOP MODES", m_teleopchooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("HEADING", Gyro.getAngle());
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoMode = m_chooser.getSelected();

    //SCHEDULE SELECTED AUTO
    switch(autoMode){
      case "STRAIGHT_LINE_AUTO": /** IF | STRAIGHT LINE AUTO | IS SELECTED */
        auto = m_robotContainer.getDriveFoward();
        if(auto != null){
          auto.schedule();
        }
        break;
      case "90_TURN_AUTO": /** IF | 90 TURN AUTO | IS SELECTED */
        auto = m_robotContainer.getFullRight();
        if(auto != null){
          auto.schedule();
        }
        break;
      case "KICK_THE_BOT_AUTO":
        auto = m_robotContainer.getKickTheBot();
        if(auto != null){
          auto.schedule();
        }
        break;
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts runn}ing. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    teleopMode = m_teleopchooser.getSelected();
    switch(teleopMode){
      case "DEFAULT DRIVE":
        m_teleopCommand = m_robotContainer.getDefaultDrive();
        if(m_teleopCommand != null){
          m_teleopCommand.schedule();
        }
      case "CURVA DRIVE":
        m_teleopCommand = m_robotContainer.getCurveDrive();
        if(m_teleopCommand != null){
          m_teleopCommand.schedule();
        }
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
