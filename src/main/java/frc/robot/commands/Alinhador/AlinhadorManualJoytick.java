package frc.robot.commands.Alinhador;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Alinhador;

public class AlinhadorManualJoytick extends Command {
    private final Alinhador alinhador;
    private final DoubleSupplier eixoJoystick;

    public AlinhadorManualJoytick(
        Alinhador alinhador,
        DoubleSupplier eixoJoystick
    ) {
        this.alinhador = alinhador;
        this.eixoJoystick = eixoJoystick;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double valor = eixoJoystick.getAsDouble();

        if (Math.abs(valor) < 0.1) {
            alinhador.parar();
        } else {
            alinhador.controleManual(valor);
        }
    }

    @Override
    public void end(boolean interrupted) {
        alinhador.parar();
    }

    @Override
    public boolean isFinished() {
        return false; // comando contínuo
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(alinhador);
    }
}
