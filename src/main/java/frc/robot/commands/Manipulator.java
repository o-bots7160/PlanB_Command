package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Manipulator
{
   public Arm     arm;
   public Shooter shooter;
   //
   //   Constructor
   //
   //
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
                 arm.setArm(false, false) ).andThen(
             new PrintCommand( "Stow complete" ) );
   }
   //
   //   Set shooter to target and shoot at speaker
   //
   //
   public Command speakerCommand( )
   {
      return new PrintCommand( "Speaker" ).andThen(
                 shooter.stow( ).andThen(
                 arm.setArm( false, false ) ).andThen(
                 shooter.speaker( ) ).andThen(
                 shooter.stow( ) ) ).unless( ()->{ return ! shooter.haveNote(); } ).andThen(
             new PrintCommand( "Speaker complete" ) );
   }
   //
   //   Set shooter to intake
   //
   //
   public Command intakeCommand( )
   {
      return new PrintCommand( "Intake" ).andThen(
                 shooter.stow( ).andThen(
                 arm.setArm( true, false ) ).andThen(
                 shooter.intake( ) ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm(false, false) ) ).unless( ()->{ return shooter.haveNote(); } ).andThen(
             new PrintCommand( "Intake complete" ) );
   }
   //
   //   Set shooter to target and shoot at amp
   //
   //
   public Command ampCommand( )
   {
      return new PrintCommand( "Amplifier" ).andThen(
                 shooter.stow( ).andThen(
                 arm.setArm( true, true ) ).andThen(
                 shooter.shootAmp( ) ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( false, false ) ) ).unless( ()->{ return ! shooter.haveNote(); } ).andThen(
             new PrintCommand( "Amplifier Complete" ) );
   }
   //
   //   Set shooter prep for climbing
   //
   //
   public Command climbCommand( )
   {
      return new PrintCommand( "Climb" ).andThen(
                 shooter.stow( ) ).andThen(
                 arm.setArm( false, true ) ).andThen(
             new PrintCommand( "Climb complete" ) );
   }
   //
   //   Set shoot to eject note
   //
   //
   public Command ejectCommand( )
   {
      return new PrintCommand( "eject" ).andThen(
                 shooter.stow( ).andThen(
                 arm.setArm( false, false ) ).andThen(
                 shooter.eject( ) ).andThen(
                 shooter.stow( ) ) ).unless( ()->{ return ! shooter.haveNote(); } ).andThen(
             new PrintCommand( "eject complete" ) );
   }
}