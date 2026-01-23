package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;

public class MoverAnguladorComhold extends Command {

    private final Angulador angulador;
    private final double alvoGraus;

    public MoverAnguladorComhold(Angulador angulador, double alvoGraus) {
        this.angulador = angulador;
        this.alvoGraus = alvoGraus;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        // Sai do hold antes de mover
        angulador.desativarHold();

        // Move para o novo ângulo
        angulador.moverParaAngulo(alvoGraus);
    }

    @Override
    public boolean isFinished() {
        // Termina quando chega perto do alvo
        return Math.abs(angulador.getAngulo() - alvoGraus)
               <= Angulador.MargenErro;
    }

    @Override
    public void end(boolean interrupted) {
        // Ativa o hold no ângulo final
        angulador.iniciarHold();
    }
}
