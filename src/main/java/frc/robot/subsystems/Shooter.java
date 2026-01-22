package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    /* ========= ENUMS ========= */

    public enum DirecaoShooter {
        PARADO,
        FRENTE,
        TRAS
    }

    public enum VelocidadeShooter {
        NORMAL(0.55),
        MEDIA(0.70),
        ALTA(0.90),
        TURBO(1.2);

        public final double valor;

        VelocidadeShooter(double valor) {
            this.valor = valor;
        }
    }

    /* ========= HARDWARE ========= */

    private final SparkMax shooterArlindo;
    private final SparkMax shooterBoquinha;

    private final RelativeEncoder arlindoEncoder;
    private final RelativeEncoder boquinhaEncoder;

    /* ========= ESTADO ========= */

    private DirecaoShooter direcaoAtual = DirecaoShooter.PARADO;
    private VelocidadeShooter velocidadeAtual = VelocidadeShooter.NORMAL;

    private static final double RPM_MIN_GIRANDO = 50.0;

    /* ========= CONSTRUTOR ========= */

    public Shooter() {

        shooterArlindo = new SparkMax(Constants.Shooter.ShooterArlindo, MotorType.kBrushed);
        shooterBoquinha = new SparkMax(Constants.Shooter.ShooterBoquinha, MotorType.kBrushed);

        arlindoEncoder = shooterArlindo.getEncoder();
        boquinhaEncoder = shooterBoquinha.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(60);

        shooterArlindo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterBoquinha.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /* ========= CONTROLE ========= */

    public void toggleFrente() {
        if (direcaoAtual == DirecaoShooter.FRENTE) {
            parar();
        } else {
            direcaoAtual = DirecaoShooter.FRENTE;
            aplicar();
        }
    }

    public void toggleTras() {
        if (direcaoAtual == DirecaoShooter.TRAS) {
            parar();
        } else {
            direcaoAtual = DirecaoShooter.TRAS;
            aplicar();
        }
    }

    public void parar() {
        direcaoAtual = DirecaoShooter.PARADO;
        aplicar();
    }

    public void setVelocidadeDireta(VelocidadeShooter nova) {
    if (velocidadeAtual != nova) {
        velocidadeAtual = nova;
        aplicar();
    }
    }

    public void setVelocidade(VelocidadeShooter nova) {
        velocidadeAtual = (velocidadeAtual == nova)
                ? VelocidadeShooter.NORMAL
                : nova;
        aplicar();
    }

    private void aplicar() {

        double base = velocidadeAtual.valor;
    
        switch (direcaoAtual) {
    
            case FRENTE:
                shooterArlindo.set(-base);
                shooterBoquinha.set(+base); 
                break;
    
            case TRAS:
                shooterArlindo.set(-base); 
                shooterBoquinha.set(-base);
                break;
    
            case PARADO:
            default:
                shooterArlindo.set(0.0);
                shooterBoquinha.set(0.0);
                break;
        }
    }
    

    /* ========= STATUS ========= */

    public boolean arlindoEstaGirando() {
        return Math.abs(arlindoEncoder.getVelocity()) > RPM_MIN_GIRANDO;
    }

    public boolean boquinhaEstaGirando() {
        return Math.abs(boquinhaEncoder.getVelocity()) > RPM_MIN_GIRANDO;
    }
    public boolean estaAtivo() {
        return direcaoAtual != DirecaoShooter.PARADO;
    }

    public DirecaoShooter getDirecaoAtual() {
        return direcaoAtual;
    }

    public VelocidadeShooter getVelocidadeAtual() {
        return velocidadeAtual;
    }

    /* ========= DASHBOARD ========= */

    @Override
    public void periodic() {
        /*SmartDashboard.putString("Shooter/Direcao", direcaoAtual.name());
        SmartDashboard.putString("Shooter/Velocidade", velocidadeAtual.name());

        SmartDashboard.putBoolean("Shooter/Ativo", direcaoAtual != DirecaoShooter.PARADO);
        SmartDashboard.putNumber("Shooter/Arlindo RPM", arlindoEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Boquinha RPM", boquinhaEncoder.getVelocity());
        SmartDashboard.putString("Shooter/Direcao", direcaoAtual.name());
        SmartDashboard.putString("Shooter/Velocidade", velocidadeAtual.name());
        SmartDashboard.putBoolean("Shooter/Ativo", direcaoAtual != DirecaoShooter.PARADO);
        */
    }
}
