package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.VelocidadeShooter;

import frc.robot.commands.Limelight.AlinhadorHorizontalAprilTag;
import frc.robot.commands.Limelight.AlinhadorVerticalAprilTag;
import frc.robot.commands.Angulador.AngularAuto;
import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorHorizontalAuto;
import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorVerticalAuto;
//import frc.robot.commands.Alinhador.AlinhadorManualJoytick;
//import frc.robot.commands.Angulador.PararAngulador;
//import frc.robot.commands.Angulador.MoverAngulador;
import frc.robot.commands.Autonomo.Tracao.AndarEncoder;
import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import frc.robot.commands.Climber.ClimberMovendo;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Angulador.MoverAnguladorComhold;


import frc.robot.commands.Shooter.*;
import frc.robot.commands.Traction.AtivarTurbo;
import frc.robot.commands.Traction.Controller;

public class RobotContainer {

    /* ===== SUBSYSTEMS ===== */
    private final Shooter shooter = new Shooter();
    private final Traction traction = new Traction();
    private final Angulador angulador = new Angulador();
    private final Limelight limelight = new Limelight();
    private final Climber climber = new Climber(); 

    /* ===== CONTROLES ===== */
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);

    /* ===== BOTÕES ===== */
    private final JoystickButton btnTurbo =
        new JoystickButton(xbox1, XboxController.Button.kA.value);

    // LIMELIGHT
    private final JoystickButton rb =
        new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    private final JoystickButton lb =
        new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);

    // SHOOTER
    private final JoystickButton btnA =
        new JoystickButton(xbox2, XboxController.Button.kA.value);


    private final JoystickButton btnX =
        new JoystickButton(xbox2, XboxController.Button.kX.value);

    private final JoystickButton btnB =
        new JoystickButton(xbox2, XboxController.Button.kB.value);

    private final JoystickButton btnY =
        new JoystickButton(xbox2, XboxController.Button.kY.value);

   /*private final JoystickButton btnY = new JoystickButton(xbox2, XboxController.Button.kY.value);
    private final JoystickButton btnB = new JoystickButton(xbox2, XboxController.Button.kB.value);*/
    //private final Trigger rt = new Trigger(() -> xbox2.getRightTriggerAxis() > 0.2);#Atualmente o movedor do shooter

    private final JoystickButton btnRb = new JoystickButton(xbox1, 6);
    private final JoystickButton btnLb = new JoystickButton(xbox1, 5);

    public RobotContainer() {
        configureBindings();

        // CONTROLE MANUAL DA TRAÇÃO (DEFAULT)
        traction.setDefaultCommand(
            new Controller(traction, xbox1)
        );
    }

    private void configureBindings() {

        /* ===== TRAÇÃO ===== */
        btnTurbo.onTrue(new AtivarTurbo(traction));

        /* ===== LIMELIGHT / APRILTAG ===== */
        btnRb.whileTrue(
            new AlinhadorHorizontalAprilTag(limelight, traction )
        );

        btnLb.whileTrue(
            new AlinhadorVerticalAprilTag(limelight, traction)
        );

        /* ===== SHOOTER ===== */
        rb.onTrue(new AtivarFrenteShooter(shooter));
        lb.onTrue(new AtivarAtrasShooter(shooter));//lb trocado para testar

        btnA.onTrue(new PararShooter(shooter));

        btnY.whileTrue(new ShooterAutoPorDistancia(shooter, limelight));
        btnB.whileTrue(new AngularAuto(angulador, limelight));
        
        btnX.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.MEDIA));
        btnB.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.ALTA));
        btnY.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.TURBO));

        climber.setDefaultCommand(
            new ClimberMovendo(climber ,() -> -xbox1.getLeftY())
        );
    
        /* ===== ALINHADOR ===== */
        //rt.onTrue(new PararAngulador(angulador));

new POVButton(xbox2, 0)
    .onTrue(new MoverAnguladorComhold(
        angulador,
        Angulador.LIMITE_SUPERIOR
    ));

new POVButton(xbox2, 270)
    .onTrue(new MoverAnguladorComhold(
        angulador,
        Angulador.LIMITE_CENTRAL
    ));

new POVButton(xbox2, 180)
    .onTrue(new MoverAnguladorComhold(
        angulador,
        Angulador.LIMITE_INFERIOR
    ));


       /*  new Trigger(() -> Math.abs(xbox2.getLeftY()) > 0.1)
            .whileTrue(
                new AlinhadorManualJoytick(
                    alinhador,
                    () -> -xbox2.getLeftY()
                )
            );*/
    }

    /* ===== AUTÔNOMO ===== */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
     new GiroPorAngulo(traction, 180),
     new AlinhadorHorizontalAuto(limelight, traction, 7),
     new AlinhadorVerticalAuto(limelight, traction, 7)
       );
    }
}
