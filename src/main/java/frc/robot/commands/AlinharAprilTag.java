package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinharAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    /* ===== CONSTANTES ===== */
    private static final int TAG_ID = 29;

    private static final double KP_ROTACAO = 0.03;
    private static final double KP_DISTANCIA = 0.6;

    private static final double TARGET_AREA = 2.5; // AJUSTE NO CAMPO
    private static final double TOL_TX = 1.0;
    private static final double TOL_AREA = 0.2;

    public AlinharAprilTag(Limelight limelight, Traction traction) {
        this.limelight = limelight;
        this.traction = traction;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0); // PIPELINE DE APRILTAG
        limelight.ligarLED();
    }

    @Override
public void execute() {

    if (!limelight.temAlvo() || limelight.getAprilTagID() != TAG_ID) {
        traction.stop();
        return;
    }

    double erroX = limelight.getTx();
    double erroArea = TARGET_AREA - limelight.getTa();

    double rotacao = erroX * KP_ROTACAO;
    double avanco = erroArea * KP_DISTANCIA;

    // CLAMP DE SEGURANÇA
    rotacao = Math.max(Math.min(rotacao, 0.5), -0.5);
    avanco  = Math.max(Math.min(avanco, 0.6), -0.6);

    // 👉 SE NÃO ESTÁ ALINHADO, NÃO ANDA
    if (Math.abs(erroX) > TOL_TX) {
        traction.arcadeMode(0.0, rotacao);
    } else {
        traction.arcadeMode(avanco, rotacao);
    }
}


    @Override
    public boolean isFinished() {
        return limelight.temAlvo()
            && limelight.getAprilTagID() == TAG_ID
            && Math.abs(limelight.getTx()) <= TOL_TX
            && Math.abs(TARGET_AREA - limelight.getTa()) <= TOL_AREA;
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }
}
