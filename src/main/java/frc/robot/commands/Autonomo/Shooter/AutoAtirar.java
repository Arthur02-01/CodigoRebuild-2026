package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.AtivarFrenteShooter;
import frc.robot.commands.Shooter.PararShooter;
import frc.robot.commands.Shooter.ShooterVelocidade;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.VelocidadeShooter;

public class AutoAtirar extends SequentialCommandGroup {

    public AutoAtirar(Shooter shooter) {

        
        addCommands(
            // Garante que começa desligado 
            new PararShooter(shooter),
            new ShooterVelocidade(shooter, VelocidadeShooter.TURBO),

            // Liga o shooter pra frente
            new AtivarFrenteShooter(shooter),

            // Tempo para pegar giro (ajuste depois em teste)
            new WaitCommand(10),

            // Tempo extra para a bola sair
            new WaitCommand(0.5),

            // Desliga tudo no final
            new PararShooter(shooter)
        );
    }
}
