package frc.robot.commands.Alinhador;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Alinhador;

public class PararAlinhador extends InstantCommand {

    public PararAlinhador(Alinhador alinhador) {
        super(alinhador::paradaEmergencia, alinhador);
    }
}