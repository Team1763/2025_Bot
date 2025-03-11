// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
//import java.util.List;
import frc.utils.GamepadUtils;
import frc.robot.subsystems.Elevator;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final Elevator elevator;

    // Current system controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  //Autonomous programs
  private SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.elevator = new Elevator();
    //configureCommands(); // use button config

    m_chooser.setDefaultOption("TestAuto", new TestAuto(m_robotDrive));
    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.resetOdometry(new Pose2d()); // from old system

    // Configure default commands
    // Throttle should be connected to the joystick slider - higher value more speed. It should affect all axis
    m_robotDrive.setDefaultCommand(
            new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getY(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getX(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getZ(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    true,
                    false),
            m_robotDrive));

    // Set the ball intake to in/out when not running based on internal state
    //m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Below are the various buttons for the joystick, change the button number to adjust which button it is connected to
    // Set swerve to X formation
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Run tube intake
    new JoystickButton(m_driverController, 1) // button 1 should be trigger
        .whileTrue(new RunCommand(() -> m_coralSubSystem.runIntakeCommand(), m_robotDrive));

    // Run tube intake in reverse
    new JoystickButton(m_driverController, 2) // button 2 should be button on top back of joystick
        .whileTrue(new RunCommand(() -> m_coralSubSystem.reverseIntakeCommand(), m_robotDrive));

    // Elevator/Arm to human player position, set ball intake to stow when idle
    new JoystickButton(m_driverController, 11) // 11 should be a button on the left side of the joystick, the top left button
        .whileTrue(new RunCommand(() -> m_coralSubSystem
        .setSetpointCommand(Setpoint.kFeederStation)
        .alongWith(m_algaeSubsystem.stowCommand()), m_robotDrive));

    // Elevator/Arm to level 2 position
    new JoystickButton(m_driverController, 14) // 14, 15, 16 should hopefully be the bottom row of buttons on the right side
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2), m_robotDrive));

    // Elevator/Arm to level 3 position
    new JoystickButton(m_driverController, 15)
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3), m_robotDrive));

    // Elevator/Arm to level 4 position
    new JoystickButton(m_driverController, 16)
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4), m_robotDrive));

    // Run ball intake, set to leave out when idle
    new JoystickButton(m_driverController, 3) // button 3 should be top left on joystick
        .whileTrue(new RunCommand(() -> m_algaeSubsystem.runIntakeCommand(), m_robotDrive));

    // Run ball intake in reverse, set to stow when idle
    new JoystickButton(m_driverController, 4) // 4 should be top right on joystick
        .whileTrue(new RunCommand(() -> m_algaeSubsystem.reverseIntakeCommand(), m_robotDrive));


        /////// Elevator control
// Run ball intake in reverse, set to stow when idle
/*new JoystickButton(m_driverController, 5) // 4 should be top right on joystick
.whileTrue(new RunCommand(() -> elevator.setMotors(-2), m_robotDrive)); // values were lowered from the one that broke the chain, but the values are unteseted

new JoystickButton(m_driverController, 6) // 4 should be top right on joystick
.whileTrue(new RunCommand(() -> elevator.setMotors(2), m_robotDrive));*/

    // Zero swerve heading
    new JoystickButton(m_driverController, 7) // 7 should be on the left side of the joystick
    .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeadingCommand(), m_robotDrive));   
  }

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    
    return m_chooser.getSelected();
  }

  /*private void configureCommands() {
    elevator.setDefaultCommand(
      elevator.GetTeleopCommand(m_driverController)
  }*/
  
}
