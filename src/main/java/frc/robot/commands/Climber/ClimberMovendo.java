package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberMovendo  extends Command{
    private final Climber climber;
    private final DoubleSupplier eixoJoystick;

public ClimberMovendo(Climber climber, DoubleSupplier eixoJoystick ){
    this.climber = climber;
    this.eixoJoystick = eixoJoystick;
    addRequirements(climber);
}
@Override
public void execute(){
    climber.moverManual(eixoJoystick.getAsDouble());
}
@Override
public void end(boolean interrupted){
    climber.moverManual(0);
}
@Override
public boolean isFinished(){
    return false;
}
}
