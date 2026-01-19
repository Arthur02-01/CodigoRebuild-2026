package frc.robot.commands.Alinhador;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Alinhador;

public class AlinhadorManual extends Command {

    private final Alinhador alinhador;
    private final DoubleSupplier velocidade;

    public AlinhadorManual(
        Alinhador alinhador,
        DoubleSupplier velocidade
    ) {
        this.alinhador = alinhador;
        this.velocidade = velocidade;
        addRequirements(alinhador);
    }

    @Override
    public void execute() {
        alinhador.controleManual(velocidade.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        alinhador.parar();
    }
}