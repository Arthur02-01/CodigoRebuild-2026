package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Limelight;

public class AngularAuto extends Command {

    private final Angulador angulador;
    private final Limelight limelight;

    // ===== ESTADOS LÓGICOS =====
    private enum EstadoAngulo {
        INFERIOR,   // 5°
        CENTRAL,    // 15°
        SUPERIOR    // 25°
    }

    private EstadoAngulo estadoAtual;

    // Histerese em metros
    private static final double HISTERESIS = 0.07;

    public AngularAuto(Angulador angulador, Limelight limelight) {
        this.angulador = angulador;
        this.limelight = limelight;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {

        double angulo = angulador.getAngulo();

        // Sincroniza estado lógico com posição real
        if (angulo >= Angulador.LIMITE_SUPERIOR - 1.0) {
            estadoAtual = EstadoAngulo.SUPERIOR;
        }
        else if (angulo >= Angulador.LIMITE_CENTRAL - 1.0) {
            estadoAtual = EstadoAngulo.CENTRAL;
        }
        else {
            estadoAtual = EstadoAngulo.INFERIOR;
        }
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            // Mantém o último ângulo conhecido
            aplicarEstado();
            return;
        }

        double distancia = limelight.getDistanciaFiltrada();

        // ===== TRANSIÇÕES COM HISTERESE =====

        // CENTRAL -> SUPERIOR (15° → 25°)
        if (estadoAtual == EstadoAngulo.CENTRAL &&
            distancia > 2.30 + HISTERESIS) {

            estadoAtual = EstadoAngulo.SUPERIOR;
        }

        // SUPERIOR -> CENTRAL (25° → 15°)
        else if (estadoAtual == EstadoAngulo.SUPERIOR &&
                 distancia < 2.30 - HISTERESIS) {

            estadoAtual = EstadoAngulo.CENTRAL;
        }

        // INFERIOR -> CENTRAL (5° → 15°)
        else if (estadoAtual == EstadoAngulo.INFERIOR &&
                 distancia > 1.55 + HISTERESIS) {

            estadoAtual = EstadoAngulo.CENTRAL;
        }

        // CENTRAL -> INFERIOR (15° → 5°)
        else if (estadoAtual == EstadoAngulo.CENTRAL &&
                 distancia < 1.55 - HISTERESIS) {

            estadoAtual = EstadoAngulo.INFERIOR;
        }

        aplicarEstado();
    }

    private void aplicarEstado() {

        switch (estadoAtual) {

            case SUPERIOR:
                angulador.moverParaAngulo(Angulador.LIMITE_SUPERIOR);
                break;

            case CENTRAL:
                angulador.moverParaAngulo(Angulador.LIMITE_CENTRAL);
                break;

            case INFERIOR:
            default:
                angulador.moverParaAngulo(Angulador.LIMITE_INFERIOR);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        angulador.iniciarHold();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
