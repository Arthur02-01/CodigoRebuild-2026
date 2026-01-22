package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Angulador;

public class MoverAngulador extends InstantCommand {

    private final Angulador angulador;
    private final double anguloAlvo;

    public MoverAngulador(Angulador angulador, double anguloAlvo) {
        this.angulador = angulador;
        this.anguloAlvo = anguloAlvo;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        angulador.moverParaAngulo(anguloAlvo);
    }
}
