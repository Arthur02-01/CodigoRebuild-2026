package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.VelocidadeShooter;
//import frc.robot.commands.Shooter.ShooterVelocidade;

public class ShooterAutoPorDistancia extends Command {

    private final Shooter shooter;
    private final Limelight limelight;

    public ShooterAutoPorDistancia(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            shooter.parar();
            return;
        }

        double distancia = limelight.getDistanciaFiltrada();

        if (distancia >= 2.30) {
            shooter.setVelocidadeDireta(VelocidadeShooter.TURBO);

        } else if (distancia >= 1.45) {
            shooter.setVelocidadeDireta(VelocidadeShooter.ALTA);

        } else {
            shooter.setVelocidadeDireta(VelocidadeShooter.MEDIA);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.parar();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
