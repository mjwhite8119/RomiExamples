// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraMount;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RomiCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CameraMountLineFollow extends SequentialCommandGroup {
  /** Creates a new CameraMountLineFollow. */
  public CameraMountLineFollow(Drivetrain drive, 
                               CameraMount cameraMount, 
                               RomiCamera camera) {

    addCommands(new PanCamera(cameraMount, 98.0),
                new TiltCamera(cameraMount, 78.0),
                new SleepCommand(1.0),
                new LineFollow(drive, camera),
                new TurnDegrees(-0.4, 180, drive),
                new LineFollow(drive, camera)
                );
  }
}
