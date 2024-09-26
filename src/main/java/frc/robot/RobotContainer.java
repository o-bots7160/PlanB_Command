// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Manipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
   // The robot's subsystems and commands are defined here...
   private final Arm     arm     = new Arm();
   private final Shooter shooter = new Shooter( ()->{ return false; }, ()->{return 0.0;} );

   private final Manipulator m_manipulator = new Manipulator( arm, shooter );

   private final CommandJoystick m_driverController = new CommandJoystick( 0 );

   /** The container for the robot. Contains subsystems, OI devices, and commands. */
   public RobotContainer() 
   {
   }
   //
   // Initialize Teleop and configure the trigger bindings
   //
   //
   public void teleopInit()
   {
      CommandScheduler.getInstance().getActiveButtonLoop().clear();
      configureTeleopBindings();
   }
   //
   // Initialize Test and configure the trigger bindings
   //
   //
   public void testInit()
   {
      CommandScheduler.getInstance().getActiveButtonLoop().clear();
      //configureTestBindingsForArm();
      configureTestBindingsForShooter();
   }
   //
   // Configure the trigger bindings for teleop mode
   //
   //
   private void configureTeleopBindings()
   {
      //
      //  Button stows manipulator
      //
      //
      new Trigger( m_driverController.button( 1 ) ).onTrue( m_manipulator.stowCommand() );
      //
      //  Button held intakes until command complete (we have note). If released stow
      //  manipulator
      //
      //
      new Trigger( m_driverController.button( 2 ) ).whileTrue( m_manipulator.intakeCommand() )
                                           .onFalse( m_manipulator.stowCommand() );
      //
      //  Button held targets speaker until command complete (we have no note). If
      //  released stow manipulator
      //
      //
      new Trigger( m_driverController.button( 3 ) ).whileTrue( m_manipulator.speakerCommand() )
                                           .onFalse( m_manipulator.stowCommand() );
      //
      //  Button held targets amplifier until command complete (we have no note). If
      //  released stow manipulator
      //
      //
      new Trigger( m_driverController.button( 4 ) ).whileTrue( m_manipulator.ampCommand() )
                                           .onFalse( m_manipulator.stowCommand() );
   }
   //
   //  Button bindings to test pneumatic arm
   //
   //
   private void configureTestBindingsForArm()
   {
      new Trigger( m_driverController.button( 1 ) ).onTrue( arm.setArm( false, false ) );
      new Trigger( m_driverController.button( 2 ) ).onTrue( arm.setArm( true,  false ) );
      new Trigger( m_driverController.button( 3 ) ).onTrue( arm.setArm( false, true  ) );
      new Trigger( m_driverController.button( 4 ) ).onTrue( arm.setArm( true,  true  ) );
   }
   //
   //  Button bindings to test shooter SysId
   //
   //
   private void configureTestBindingsForShooter()
   {
      new Trigger( m_driverController.button( 1 ) ).whileTrue( arm.setArm( true,  false).andThen( shooter.setAngleSysIdTest( ) ) );
      new Trigger( m_driverController.button( 2 ) ).whileTrue( shooter.setTopSysIdTest( ) );
      new Trigger( m_driverController.button( 2 ) ).whileTrue( shooter.setBotSysIdTest( ) );
      new Trigger( m_driverController.button( 4 ) ).whileTrue( arm.setArm( true,  true  ) );
   }
   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand()
   {
      // An example command will be run in autonomous
      return m_manipulator.stowCommand(); // Autos.exampleAuto(m_exampleSubsystem);
   }
}
