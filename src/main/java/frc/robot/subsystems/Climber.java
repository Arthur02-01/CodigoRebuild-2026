package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private final SparkMax motorClimber;
    private final SparkMax motorFollower;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;
    private final ArmFeedforward feedforward;


    private static final double DIAMETRO_CLIMBER = 0.30;
    private static final double CIRCUNFERENCIA = Math.PI * DIAMETRO_CLIMBER;

    public static final double LIMITE_INFERIOR = 0.0;
    public static final double LIMITE_SUPERIOR = 1.15;

    public double POSICAO_ANTERIOR = 0.0;
    public double TEMPO_SEM_MOVIMENTO = 0.0;
    public boolean emFalha = false;

    public final double DT = 0.02;
    public final double VELOCIDADE_MIN = 0.002;
    public final double TEMPO_MAX_TRAVADO = 0.4;
    public final double ERRO_TOLERANCIA = 0.3;

    private double alvoAltura = 0.0;

    
    public Climber() {

        motorClimber = new SparkMax(
            Constants.ClimberConstants.ClimberMotor,
            MotorType.kBrushless
        );

        motorFollower = new SparkMax(
            Constants.ClimberConstants.ClimberMotor2,
            MotorType.kBrushless
        );

        encoder = motorClimber.getEncoder();
        pid = motorClimber.getClosedLoopController();

        
        SparkMaxConfig masterCfg = new SparkMaxConfig();
        masterCfg
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);

        masterCfg.closedLoop
            .p(0.4)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0);

       
        SparkMaxConfig followerCfg = new SparkMaxConfig();
        followerCfg
            .follow(motorClimber, false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);

        motorClimber.configure(
            masterCfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        motorFollower.configure(
            followerCfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        feedforward = new ArmFeedforward(
            Constants.ClimberConstants.kS,
            Constants.ClimberConstants.kG,
            Constants.ClimberConstants.kV
        );
    }

    public void irParaAltura(double alturaMetros) {

    alturaMetros = MathUtil.clamp(
        alturaMetros,
        LIMITE_INFERIOR,
        LIMITE_SUPERIOR
    );

    alvoAltura = alturaMetros;

    double rotacoes = alturaParaRotacoes(alturaMetros);

    pid.setReference(
        rotacoes,
        SparkBase.ControlType.kPosition
    );
    }

    public void zerarEncoder() {
        encoder.setPosition(0.0);
    }

    public void moverManual(double velocidade){
        velocidade = MathUtil.clamp(velocidade, -0.8, 0.8);

        if (velocidade < 0 && encoder.getPosition() <= 0){
            velocidade = 0;
        }
        if(velocidade > 0 && rotacoesParaAltura(encoder.getPosition())>= LIMITE_SUPERIOR){
             velocidade = 0;
        }
        motorClimber.set(velocidade);
    }


    private double alturaParaRotacoes(double alturaMetros) {
        return alturaMetros / CIRCUNFERENCIA;
    }

    public double rotacoesParaAltura(double rotacoes) {
        return rotacoes * CIRCUNFERENCIA;
    }
    
    private void entrarEmFalha(String Motivo){
        emFalha = true;
        motorClimber.stopMotor();
        SmartDashboard.putString("Climber/Falha", Motivo);
    }


    @Override
public void periodic() {

    double posicaoAtual = encoder.getPosition();
    double velocidade = (posicaoAtual - POSICAO_ANTERIOR) / DT;

    double erro =
        Math.abs(alvoAltura - rotacoesParaAltura(posicaoAtual));

    boolean tentandoMover = erro > ERRO_TOLERANCIA;

    if (tentandoMover) {

        if (Math.abs(velocidade) < VELOCIDADE_MIN) {
            TEMPO_SEM_MOVIMENTO += DT;
        } else {
            TEMPO_SEM_MOVIMENTO = 0.0;
        }

    } else {
        TEMPO_SEM_MOVIMENTO = 0.0; // chegou no alvo
    }

    POSICAO_ANTERIOR = posicaoAtual;

    if (TEMPO_SEM_MOVIMENTO >= TEMPO_MAX_TRAVADO) {
        entrarEmFalha("Climber travado ou sensor falhou");
    }

    SmartDashboard.putNumber(
        "Climber/Altura (m)",
        rotacoesParaAltura(posicaoAtual)
    );

    SmartDashboard.putNumber(
        "Climber/Erro (m)",
        erro
    );
}
}