package frc.robot.subsystems;

// Imports da REV (Spark MAX)
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

// Gyro
import com.ctre.phoenix6.hardware.Pigeon2;

// Imports da WPILib
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Declara o subsistema de tração
public class Traction extends SubsystemBase {

    // VARIÁVEIS
    public boolean turbo;

    // Motores
    private SparkMax rightMotorFront =
            new SparkMax(Constants.TractionConstants.rightFrontMotorID, MotorType.kBrushed);

    private SparkMax rightMotorBack =
            new SparkMax(Constants.TractionConstants.rightBackMotorID, MotorType.kBrushed);

    private SparkMax leftMotorFront =
            new SparkMax(Constants.TractionConstants.leftFrontMotorID, MotorType.kBrushed);

    private SparkMax leftMotorBack =
            new SparkMax(Constants.TractionConstants.leftBackMotorID, MotorType.kBrushed);

    // Encoders do Spark
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // Gyro
    private final Pigeon2 pigeon = new Pigeon2(22); // id 22 can

    // Configurações
    private SparkMaxConfig configSparkMotorEsquerda = new SparkMaxConfig();
    private SparkMaxConfig configSparkMotorDireita = new SparkMaxConfig();

    // Grupos de motores
    @SuppressWarnings("removal")
    private MotorControllerGroup leftMotorControllerGroup =
            new MotorControllerGroup(leftMotorFront, leftMotorBack);

    @SuppressWarnings("removal")
    private MotorControllerGroup rightMotorControllerGroup =
            new MotorControllerGroup(rightMotorFront, rightMotorBack);

    // Drive
    private DifferentialDrive differentialDrive =
            new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

    // Constantes físicas
    private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 polegadas
    private static final double METERS_PER_ROTATION =
            Math.PI * WHEEL_DIAMETER_METERS;

    // Ajuste do robô
    private static final double GEAR_RATIO = 10.71;

    // CONSTRUTOR
    public Traction() {

        pigeon.reset();

        // Configuração motores da direita
        configSparkMotorDireita
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);

        rightMotorFront.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightMotorBack.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configuração motores da esquerda
        configSparkMotorEsquerda
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);

        leftMotorFront.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        leftMotorBack.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Inicializa encoders do Spark
        leftEncoder = leftMotorFront.getEncoder();
        rightEncoder = rightMotorFront.getEncoder();
    }

    // CONTROLE DE MOVIMENTO

    public void arcadeMode(double drive, double turn) {
        differentialDrive.arcadeDrive(-drive, +turn);
    }

    public void stop() {
        differentialDrive.stopMotor();
    }

    public void ativarTurbo(boolean turbo) {
        this.turbo = turbo;
    }

    // GYRO


    public void resetYaw() {
        pigeon.reset();
    }

    public double getYaw() {
        return pigeon.getAngle(); // contínuo
    }

    // ENCODERS

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getAverageDistance() {
        double leftDistance =
            (leftEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;

        double rightDistance =
            (-rightEncoder.getPosition() / GEAR_RATIO) * METERS_PER_ROTATION;

        // Usa o maior valor absoluto para mais estabilidade
        return Math.max(Math.abs(leftDistance), Math.abs(rightDistance));
    }
}