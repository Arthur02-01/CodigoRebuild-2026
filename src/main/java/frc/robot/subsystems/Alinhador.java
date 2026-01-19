package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Alinhador extends SubsystemBase {

    private final SparkMax AlinhadorMotor;
    private final RelativeEncoder AlinhadorEncoder;
    private final SparkClosedLoopController pid;
    private final ArmFeedforward feedforward;

    // Limites mecânicos (graus)
    public static final double LIMITE_SUPERIOR = 85.0;
    public static final double LIMITE_INFERIOR = 5.0;
    public static final double MargenErro = 2.0;
    private double anguloHold = 0.0;
    private boolean holdAtivo = false;


    // Redução do sistema
    private static final double REDUCAO = 12.0;

    // Controle manual
    private static final double VELOCIDADE_MAX = 0.15;

    public Alinhador() {

        AlinhadorMotor = new SparkMax(
            Constants.Alinhador.AlinhadorMotor,
            MotorType.kBrushless
        );

        AlinhadorEncoder = AlinhadorMotor.getEncoder();
        pid = AlinhadorMotor.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .inverted(false)
           .smartCurrentLimit(60);

        cfg.closedLoop
           .p(0.3)
           .i(0.0)
           .d(0.0)
           .outputRange(-1.0, 1.0);

        AlinhadorMotor.configure(
            cfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        feedforward = new ArmFeedforward(
            Constants.FFAlinhador.kS,
            Constants.FFAlinhador.kG,
            Constants.FFAlinhador.kV
        );
    }
    public double getAngulo() {
        return AlinhadorEncoder.getPosition() * (360.0 / REDUCAO);
    }

    public double getAnguloRad() {
        return Math.toRadians(getAngulo());
    }
    public double getAlinhadorPosition(){
    return AlinhadorEncoder.getPosition();
    }
    private double grausParaRotacao(double graus) {
        return (graus / 360.0) * REDUCAO;
    }
    public boolean noLimiteSuperior(){
    return getAngulo() >= (LIMITE_SUPERIOR - MargenErro);
    }
    public boolean noLimiteInferior(){
    return getAngulo() <= (LIMITE_INFERIOR + MargenErro);
    }
    // Zona onde começa a desacelerar (graus)
private static final double ZONA_DESACELERACAO = 6.0;

private double aplicarZonaDesaceleracao(double velocidade) {

    double angulo = getAngulo();

    // Subindo (em direção ao limite superior)
    if (velocidade > 0 && angulo >= (LIMITE_SUPERIOR - ZONA_DESACELERACAO)) {
        double fator =
            (LIMITE_SUPERIOR - angulo) / ZONA_DESACELERACAO;
        return velocidade * MathUtil.clamp(fator, 0.0, 1.0);
    }

    // Descendo (em direção ao limite inferior)
    if (velocidade < 0 && angulo <= (LIMITE_INFERIOR + ZONA_DESACELERACAO)) {
        double fator =
            (angulo - LIMITE_INFERIOR) / ZONA_DESACELERACAO;
        return velocidade * MathUtil.clamp(fator, 0.0, 1.0);
    }

    return velocidade;
}


    public void moverParaAngulo(double alvoGraus) {

        alvoGraus = MathUtil.clamp(
            alvoGraus,
            LIMITE_INFERIOR,
            LIMITE_SUPERIOR
        );

        double ffVolts = feedforward.calculate(
            Math.toRadians(alvoGraus),
            0.0
        );

        pid.setSetpoint(
    grausParaRotacao(alvoGraus),
    ControlType.kPosition,
    ClosedLoopSlot.kSlot0,
    ffVolts
);
    }

    public void controleManual(double velocidade) {
    
        // Hard stop absoluto
        if (noLimiteInferior() && velocidade < 0) {
            AlinhadorMotor.set(0.0);
            return;
        }
    
        if (noLimiteSuperior() && velocidade > 0) {
            AlinhadorMotor.set(0.0);
            return;
        }
    
        // Sai do hold ao mover manualmente
        desativarHold();
    
        // Clamp básico
        velocidade = MathUtil.clamp(
            velocidade,
            -VELOCIDADE_MAX,
            VELOCIDADE_MAX
        );
    
        // Zona de desaceleração
        velocidade = aplicarZonaDesaceleracao(velocidade);
    
        // Feedforward contra a gravidade
        double ffVolts = feedforward.calculate(
            getAnguloRad(),
            0.0
        );
    
        double ffPercent = ffVolts / 12.0;
    
        AlinhadorMotor.set(velocidade + ffPercent);
    }
    
    public void paradaEmergencia() {
        // Cancela qualquer referência de closed-loop
        pid.setSetpoint(
            AlinhadorEncoder.getPosition(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            0.0
        );
    
        // Garante saída zero
        AlinhadorMotor.set(0.0);
    }

    public void parar() {
        AlinhadorMotor.set(0.0);
    }
    public void iniciarHold() {
        anguloHold = getAngulo();
        holdAtivo = true;
    }
    
    public void desativarHold() {
        holdAtivo = false;
    }
    
    @Override
public void periodic() {
    if (holdAtivo) {
        moverParaAngulo(anguloHold);
    }
    SmartDashboard.putNumber(
        "Alinhador/Angulo (graus)",
        getAngulo()
    );

    SmartDashboard.putNumber(
        "Alinhador/Encoder (rotacoes)",
        AlinhadorEncoder.getPosition()
    );

    SmartDashboard.putBoolean(
        "Alinhador/Limite Superior",
        noLimiteSuperior()
    );

    SmartDashboard.putBoolean(
        "Alinhador/Limite Inferior",
        noLimiteInferior()
    );

    SmartDashboard.putNumber(
        "Alinhador/Corrente (A)",
        AlinhadorMotor.getOutputCurrent()
    );

    SmartDashboard.putBoolean(
        "Alinhador/Hold Ativo",
        holdAtivo
    );

    SmartDashboard.putNumber(
        "Alinhador/Hold Angulo",
        anguloHold
    );

    double ffVolts = feedforward.calculate(
        getAnguloRad(),
        0.0
    );

    SmartDashboard.putNumber(
        "Alinhador/FeedForward (V)",
        ffVolts
    );

    SmartDashboard.putNumber(
        "Alinhador/Motor Output",
        AlinhadorMotor.get()
    );
}

}

