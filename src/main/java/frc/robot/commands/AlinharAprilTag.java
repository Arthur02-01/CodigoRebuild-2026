package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinharAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    private static final double KP_ROTACAO   = 0.05;
    private static final double KP_DISTANCIA = 0.6;

    private static final double TARGET_AREA = 2.5;

    public AlinharAprilTag(Limelight limelight, Traction traction) {
        this.limelight = limelight;
        this.traction = traction;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.ligarLED();
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        double erroX    = limelight.getTx();
double erroArea = TARGET_AREA - limelight.getTa();

double rotacao = erroX * KP_ROTACAO;
double avanco  = erroArea * KP_DISTANCIA;

// Limites
rotacao = Math.max(Math.min(rotacao, 0.5), -0.5);
avanco  = Math.max(Math.min(avanco, 0.5), -0.5);

// Nunca zera totalmente o avanço
double fatorAlinhamento =
    Math.max(0.3, 1.0 - Math.abs(erroX) / 20.0);

avanco *= fatorAlinhamento;

traction.arcadeMode(avanco, rotacao);
    }

    @Override
    public boolean isFinished() {
        return false; // termina somente ao soltar o botão
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }
}
