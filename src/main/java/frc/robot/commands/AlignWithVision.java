// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithVision extends PIDCommand {
  private int counter = 0;
  private boolean canFinishCommand = false;

  private Drivetrain drive;
  private Vision vision;

  /** Creates a new TurnWithVision. */
  public AlignWithVision(Drivetrain drive, Vision vision) {
    super(
        // The controller that the command will use
        vision.getController(),
        // This should return the measurement
        () -> vision.getTarget().getCenterX(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          if(output > 1.0) {
            output = 1.0;
          }
          if(output < -1.0) {
            output = -1.0;
          }

          drive.arcadeDrive(0, -output);
          SmartDashboard.putNumber("PID Output", output);
        });

    this.drive = drive;
    this.vision = vision;
      
    addRequirements(drive, vision);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    super.execute();
    drive.feed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter % 20 == 0) {
      canFinishCommand = true;
    } else {
      canFinishCommand = false;
    }
    counter++;
    //return canFinishCommand && getController().atSetpoint();
    return false;
  }
}
