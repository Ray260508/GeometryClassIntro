// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Represents a translation in 2D/3D space. This object can be used to represent a point or a vector.
    Translation2d translation2d = new Translation2d(1.323, 2.718);
    Translation3d translation3d = new Translation3d(0.09, 0.0, 0.235);

    // Represents a transformation for a Pose3d in the pose's frame.
    // here is a common example(for vision system)
    Transform3d robotToCamera = new Transform3d(translation3d, new Rotation3d(0, 0, 0));

    // Represents a 2D/3D pose containing translational and rotational elements.
    // we can use for representing robot pose, field(AprilTag or specific scoring pose)
    Pose2d robotPose = new Pose2d(new Translation2d(1.25, 3.14), Rotation2d.fromDegrees(30));
    Pose3d robotPose3d = new Pose3d(translation3d, new Rotation3d(0, 0, 0));

    // A rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).
    // Rotation2d supports conversion between degrees, radians, and rotations.
    Rotation2d rotation2d = new Rotation2d(90);
    double degrees = rotation2d.getDegrees();
    Rotation2d jointRotation = rotation2d.plus(Rotation2d.fromDegrees(90)); 
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
