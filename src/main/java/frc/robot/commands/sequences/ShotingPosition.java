// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RunPivot;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShotingPosition extends SequentialCommandGroup {
  /** Creates a new ShotingPosition. */
  public ShotingPosition(Pivot pivot, double setPoint, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunPivot(pivot, setPoint).alongWith(new RunShooter(shooter, ShooterConstants.kShootingPower))
    );
  }
}
