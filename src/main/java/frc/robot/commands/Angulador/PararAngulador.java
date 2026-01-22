package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Angulador;

public class PararAngulador extends InstantCommand {

    public PararAngulador(Angulador alinhador) {
        super(alinhador::paradaEmergencia, alinhador);
    }
}