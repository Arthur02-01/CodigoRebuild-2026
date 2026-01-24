package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;

public class MoverAnguladorComhold extends Command {

    private final Angulador angulador;
    private final double alvo;
    private boolean executar = true;

    public MoverAnguladorComhold(Angulador angulador, double alvo) {
        this.angulador = angulador;
        this.alvo = alvo;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {

        double anguloAtual = angulador.getAngulo();

        if (Math.abs(anguloAtual - alvo) <= Angulador.MargenErro) {
            executar = false;
            return;
        }

        angulador.moverParaAngulo(alvo);
    }

    @Override
    public boolean isFinished() {
        if (!executar) return true;

        return Math.abs(angulador.getAngulo() - alvo)
               <= Angulador.MargenErro;
    }

    @Override
    public void end(boolean interrupted) {
        if (executar) {
            angulador.iniciarHold();
        }
    }
}