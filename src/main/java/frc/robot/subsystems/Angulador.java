package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Alinhador;

public class Angulador extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;
    private final ArmFeedforward feedforward;

    public static final double LIMITE_SUPERIOR = 55.0;
    public static final double LIMITE_CENTRAL = 25.0;
    public static final double LIMITE_INFERIOR = 10.0;
    public static final double MargenErro = 2.5;

    public double POSICAO_ANTERIOR = 0.0;
    public double TEMPO_SEM_MOVIMENTO = 0.0;
    public boolean emFalha = false;

    public static final double DT = 0.02;
    public static final double VELOCIDADE_MIN_GRAUS = 0.01; // graus/s
    public static final double TEMPO_MAX_TRAVADO = 0.8;     // s
    public static final double ERRO_TOLERANCIA = 1.5;       // graus

    private boolean holdAtivo = false;
    private double anguloHold = 0.0;

    private static final double REDUCAO = 5.0;

    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(90.0, 180.0);

    private TrapezoidProfile.State goal =
        new TrapezoidProfile.State(0.0, 0.0);

    private TrapezoidProfile.State setpoint =
        new TrapezoidProfile.State(0.0, 0.0);

    private boolean perfilAtivo = false;

    public Angulador() {

        motor = new SparkMax(
            Constants.Alinhador.AlinhadorMotor,
            SparkLowLevel.MotorType.kBrushless
        );

        encoder = motor.getEncoder();
        pid = motor.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(25);

        cfg.closedLoop
           .p(0.8)
           .i(0.0)
           .d(0.001);

        motor.configure(
            cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        feedforward = new ArmFeedforward(
            Constants.FFAlinhador.kS,
            Constants.FFAlinhador.kG,
            Constants.FFAlinhador.kV
        );
    }

    public double getAngulo() {
        return encoder.getPosition() * (360.0 / REDUCAO);
    }

    private double grausParaRotacao(double graus) {
        return (graus / 360.0) * REDUCAO;
    }

    public void moverParaAngulo(double graus) {

    graus = MathUtil.clamp(graus, LIMITE_INFERIOR, LIMITE_SUPERIOR);

    if (jaEstaNoAlvo(graus)) {
        iniciarHold();
        return;
    }

    desativarHold();
    goal = new TrapezoidProfile.State(graus, 0.0);
    perfilAtivo = true;
}

    
    private boolean jaEstaNoAlvo(double alvo) {
    return Math.abs(getAngulo() - alvo) <= MargenErro;
    }

     private void entrarEmFalha(String motivo) {
    emFalha = true;
    perfilAtivo = false;

    // entra em hold no ângulo atual
    anguloHold = getAngulo();
    holdAtivo = true;

    SmartDashboard.putString("Angulador/Falha", motivo);
}

    public void parar() {
        perfilAtivo = false;
        motor.set(0.0);
    }
    public void iniciarHold() {
        anguloHold = getAngulo();
        holdAtivo = true;
        perfilAtivo = false;
    }

    public void desativarHold() {
    holdAtivo = false;
}

    @Override
public void periodic() {

    
    double anguloAtual = getAngulo();
    double velocidadeGraus = (anguloAtual - POSICAO_ANTERIOR) / DT;

    double erro = Math.abs(goal.position - anguloAtual);
    boolean tentandoMover = erro > ERRO_TOLERANCIA;

        if (tentandoMover) {
            if (Math.abs(velocidadeGraus) < VELOCIDADE_MIN_GRAUS) {
                TEMPO_SEM_MOVIMENTO += DT;
            } else {
                TEMPO_SEM_MOVIMENTO = 0.0;
            }
        } else {
            TEMPO_SEM_MOVIMENTO = 0.0;
        }

        POSICAO_ANTERIOR = anguloAtual;

        boolean quaseNoAlvo = erro < 3.0; // graus

        if (TEMPO_SEM_MOVIMENTO >= TEMPO_MAX_TRAVADO) {
            if (quaseNoAlvo) {
                iniciarHold(); // finaliza mesmo assim
            } else {
                entrarEmFalha("Angulador travado");
            }
        }

        

    if (perfilAtivo) {

        TrapezoidProfile profile =
            new TrapezoidProfile(constraints);

        setpoint = profile.calculate(0.02, setpoint, goal);

        double ffVolts = feedforward.calculate(
            Math.toRadians(setpoint.position),
            Math.toRadians(setpoint.velocity)
        );

        pid.setSetpoint(
            grausParaRotacao(setpoint.position),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );

        if (Math.abs(goal.position - setpoint.position) <= MargenErro &&
            Math.abs(setpoint.velocity) < 1.0) {
            iniciarHold();
        }
    }

    if (holdAtivo) {

        double ffVolts = feedforward.calculate(
            Math.toRadians(anguloHold),
            0.0
        );

        pid.setSetpoint(
            grausParaRotacao(anguloHold),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
    }

    SmartDashboard.putNumber("Angulador/Angulo", getAngulo());
}
}
