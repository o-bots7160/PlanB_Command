package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Manipulator
{
   public Arm     arm;
   public Shooter shooter;
   public Manipulator( Arm new_arm, Shooter new_shooter )
   {
      arm     = new_arm;
      shooter = new_shooter;
   }
   //
   //   Command to Stow manipulator
   //
   //
   public Command stowCommand( )
   {
      return new PrintCommand( "Stow" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm(false, false) );
   }
   //
   //   Set shooter to target and shoot at speaker
   //
   //
   public Command speakerCommand( )
   {
      return new PrintCommand( "Speaker" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( false, false ) ).andThen(
                 shooter.shootSpeaker( ) ).andThen(
                 shooter.stow( ) );
   }
   //
   //   Set shooter to intake
   //
   //
   public Command intakeCommand( )
   {
      return new PrintCommand( "Intake" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( true, false ) ).andThen(
                 shooter.intake( ) ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm(false, false) ).until( ()->{ return shooter.haveNote(); } );
   }
   //
   //   Set shooter to target and shoot at amp
   //
   //
   public Command ampCommand( )
   {
      return new PrintCommand( "Amplifier" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( true, true ) ).andThen(
                 shooter.shootAmp( ) ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( false, false ) );
   }
   //
   //   Set shooter prep for climbing
   //
   //
   public Command climbCommand( )
   {
      return new PrintCommand( "Amplifier" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( true, true ) );
   }
}