package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase
{
   private TimedDoubleSolenoid _extension = new TimedDoubleSolenoid ( 0, 1, 1.0, 1.0 );
   private TimedDoubleSolenoid _arm0      = new TimedDoubleSolenoid ( 2, 3, 1.0, 1.0 );
   private TimedDoubleSolenoid _arm1      = new TimedDoubleSolenoid ( 4, 5, 1.0, 1.0 );

   private boolean target_extend = false; // start retracted
   private boolean target_elbow  = false; // start down
   //
   //   Checks if elbow is at position, if not move it there
   //
   //
   private boolean elbowIsAtPosition( )
   {
      boolean  return_value = true;

      if ( (  target_elbow && !_arm0.isExtended( ) ) ||
           ( !target_elbow && !_arm0.isRetracted( ) ) ) // elbow needs to change
      {
         if ( !_extension.isRetracted( ) ) // and the extendsion isn't pulled in
         {
            _extension.retract( );         // pull it in
         }
         else // extension is in and we can raise or lower the arm
         {
            if ( target_elbow )
            {
               _arm0.extend( );
               _arm1.extend( );
            }
            else
            {
               _arm0.retract( );
               _arm1.retract( );
            }
         }
         return_value = false;
      }
      return return_value;
   }
   //
   //   Checks if extension is at position, if not move it there
   //
   //
   private boolean extensionIsAtPosition( )
   {
      boolean  return_value = true;

      if ( (  target_extend && !_extension.isExtended( )  ) ||
           ( !target_extend && !_extension.isRetracted( ) ) )  // the exension is not where we want
      {
         if ( target_extend )
         {
            _extension.extend( );  // push it out
         }
         else
         {
            _extension.retract( ); // pull it in
         }
         return_value = false;
      }
      return return_value;
   }
   //
   //   Set arm elbow and extension
   //
   //
   public Command setArm( boolean new_extend, boolean new_elbow )
   {
      return new FunctionalCommand(
                ()->{ target_extend = new_extend;
                      target_elbow  = new_elbow; },
                ()->{ },
                interrupted ->{ },
                ()->{ return isAtPosition( ); },
                this );
   }
   //
   //   Check to see if the arm is in position
   //
   //
   public boolean isAtPosition( )
   {
      boolean return_value = false;

      if ( elbowIsAtPosition( ) )
      {
         if ( extensionIsAtPosition( ) )
         {
            return_value = true;
         }
      }
      return return_value;
   }
   //
   //   Check to see if the arm is in position
   //
   //
   @Override
   public void periodic( )
   {
      isAtPosition( );
   }
}