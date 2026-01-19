package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Alinhador;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.VelocidadeShooter;
import frc.robot.commands.Alinhador.PararAlinhador;
import frc.robot.commands.Autonomo.Tracao.AndarEncoder;
import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlinharAprilTag;
import frc.robot.commands.Alinhador.AlinhadorManualJoytick;



import frc.robot.commands.Shooter.*;
import frc.robot.commands.Traction.AtivarTurbo;
import frc.robot.commands.Traction.Controller;

public class RobotContainer {


    /* ===== SUBSYSTEM ===== */
    private final Shooter shooter = new Shooter();
    public final Traction traction = new Traction();
    private final Alinhador alinhador = new Alinhador();
    private final Limelight limelight = new Limelight();
        /* ===== CONTROLE ===== */
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);

    /* ===== BOTÕES ===== */
    private final JoystickButton btnTurbo = new JoystickButton(xbox1, 1);

    private final JoystickButton rb =
            new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    private final JoystickButton lb =
            new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);

    private final JoystickButton btnA =
            new JoystickButton(xbox2, XboxController.Button.kA.value);

    private final JoystickButton btnX =
            new JoystickButton(xbox2, XboxController.Button.kX.value);

    private final JoystickButton btnB =
            new JoystickButton(xbox2, XboxController.Button.kB.value);

    private final JoystickButton btnY =
            new JoystickButton(xbox2, XboxController.Button.kY.value);
    private final Trigger rt =
            new Trigger(() -> xbox2.getRightTriggerAxis() > 0.2);
private final JoystickButton btnAlinhar =
    new JoystickButton(xbox1, XboxController.Button.kX.value);

        

    public RobotContainer() {
        configureBindings();
        traction.setDefaultCommand(
        new Controller(traction, xbox1)
    );
    }

    private void configureBindings() {

        btnTurbo.onTrue(new AtivarTurbo(traction));
        /* ===== DIREÇÃO ===== */
        rb.onTrue(new AtivarFrenteShooter(shooter));
        lb.onTrue(new AtivarAtrasShooter(shooter));

        /* ===== PARAR ===== */
        btnA.onTrue(new PararShooter(shooter));

        /* ===== VELOCIDADES ===== */
        btnX.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.MEDIA));
        btnB.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.ALTA));
        btnY.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.TURBO));
        rt.onTrue(new PararAlinhador(alinhador));
        btnAlinhar.whileTrue(
    new AlinharAprilTag(limelight, traction)
);


        new Trigger(() -> Math.abs(xbox2.getLeftY()) > 0.1)
    .whileTrue(
        new AlinhadorManualJoytick(
            alinhador,
            () -> -xbox2.getLeftY()
        )
    );

    
    }
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new AndarEncoder(traction, 0.6, 0.5),
            new GiroPorAngulo(traction, 90).withTimeout(0.55),
            new AndarEncoder(traction, 0.6, 0.3),
            new AndarEncoder(traction, -0.6, 0.3),
            new GiroPorAngulo(traction, 90).withTimeout(0.43),
            new AndarEncoder(traction, 0.6, 0.4)
        );
    }
}
