package frc.robot.commands.Alinhador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Alinhador;

public class AlinhadorIrParaAngulo extends Command {

    private final Alinhador alinhador;
    private final double alvo;

    public AlinhadorIrParaAngulo(Alinhador alinhador, double alvoGraus) {
        this.alinhador = alinhador;
        this.alvo = alvoGraus;
        addRequirements(alinhador);
    }

    @Override
    public void execute() {
        alinhador.moverParaAngulo(alvo);
    }

    @Override
    public boolean isFinished() {
        return false; // mantém segurando o braço
    }

    @Override
    public void end(boolean interrupted) {
        alinhador.parar();
    }
}
