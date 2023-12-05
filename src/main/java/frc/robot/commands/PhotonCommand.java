// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonCommand extends CommandBase {
  /** Creates a new PhotonCommand. */
  // private PhotonSubsystem m_photonSubsystem;
  private final PhotonSubsystem m_photonSubsystem = PhotonSubsystem.getInstance();
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private Trigger m_x;
  // PhotonSubsystem movementObj = new PhotonSubsystem();
  SwerveSubsystem moveObj;

  public PhotonCommand(Trigger x) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_x = x;
    addRequirements(m_photonSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_x.getAsBoolean()) {
      swerve.drive(
        m_photonSubsystem.xPID.calculate(m_photonSubsystem.verticalDistance(), 1),
        m_photonSubsystem.yPID.calculate(m_photonSubsystem.horizontalDistance(), 0), 
        // m_photonSubsystem.zPID.calculate(m_photonSubsystem.zAngle(), 180),
        0,
        true);
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