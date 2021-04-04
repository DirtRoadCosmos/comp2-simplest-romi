// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTime extends CommandBase {
  private final double m_duration;
  private final double m_speed;
  private final Drivetrain m_drive;
  private long m_startTime;
  private OnBoardIO io;
  private int mode = 0;
  private int ticksSinceLastModeSwitch = 0;
  private int changeMode = 0;
  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param time How much time to drive in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveTime(double speed, double time, Drivetrain drive) {
    m_speed = speed;
    m_duration = time * 1000;
    m_drive = drive;
    io = new OnBoardIO();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_drive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int leftIR = io.getLeftValue();
    int middleIR = io.getMiddleValue();
    int rightIR = io.getRightValue();
    ticksSinceLastModeSwitch++;

    if (middleIR < 3000 && leftIR < 3200 && rightIR < 3500){ // 3 sensor hit
      if (ticksSinceLastModeSwitch > 30) {  // ok to switch mode
        if (changeMode >= 2){
          m_drive.tankDrive(0.0, 0.0);
          mode = 2;
        }
        else if (mode == 0){
          mode = 1;
        }else if (mode == 1){
          mode = 0;
        }
        ticksSinceLastModeSwitch = 0;
        changeMode++;
      }
    }

    if (middleIR > 3000 && leftIR < 3200 && rightIR < 3500){ // 2 sensor hit
      m_drive.tankDrive(0.0, 0.0);
      mode = 2;
    }
    

    System.out.println(leftIR + "-" + middleIR + "-" + rightIR + "-" + mode + "-" + changeMode); 
    if (mode == 0){
      if (middleIR < 3000) {
        m_drive.tankDrive(.52, .5);
      }else if (leftIR < 3200) {
        m_drive.tankDrive(.3, .5);
      } else if (rightIR < 3500) {
        m_drive.tankDrive(.5, .3);
      } else {
        m_drive.tankDrive(.42, .4);
      }

    }else if (mode == 1){
      m_drive.tankDrive(.4, .8);
    } else if (mode > 1) {
      m_drive.tankDrive(0, 0);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
