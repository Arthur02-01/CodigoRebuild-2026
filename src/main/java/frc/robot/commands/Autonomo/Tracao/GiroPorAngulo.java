package frc.robot.commands.Autonomo.Tracao;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Traction;

public class GiroPorAngulo extends Command {

    private final Traction traction;
    private final double anguloAlvo;
    private final double velocidade = 0.75;

    private double anguloInicial;

    public GiroPorAngulo(Traction traction, double anguloAlvo) {
        this.traction = traction;
        this.anguloAlvo = anguloAlvo;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        // Salva o ângulo atual — NÃO zera o gyro
        anguloInicial = traction.getYaw();
    }

    @Override
    public void execute() {
        traction.arcadeMode(0, -velocidade);
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
    }

    @Override
    public boolean isFinished() {
        double anguloAtual = traction.getYaw();
        return Math.abs(anguloAtual - anguloInicial) >= anguloAlvo;
    }
}