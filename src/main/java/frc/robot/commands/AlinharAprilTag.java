package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinharAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    /* ===== CONSTANTES DE CONTROLE ===== */

    private static final double KP = 0.03;
    private static final double ERRO_TOLERANCIA = 1.0;

    public AlinharAprilTag(Limelight limelight, Traction traction) {
        this.limelight = limelight;
        this.traction = traction;

        addRequirements(traction);
    }

    @Override
    public void initialize() {
        limelight.ligarLED();
    }

    @Override
    public void execute() {

        // Sem alvo → não faz nada
        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        double erro = limelight.getTx();

        // Dentro da tolerância → alinhado
        if (Math.abs(erro) <= ERRO_TOLERANCIA) {
            traction.stop();
            return;
        }

        // Controle proporcional simples
        double giro = KP * erro;

        traction.arcadeMode(0.0, giro);
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }

    @Override
public boolean isFinished() {
    return limelight.temAlvo()
        && Math.abs(limelight.getTx()) <= ERRO_TOLERANCIA;
}
}
