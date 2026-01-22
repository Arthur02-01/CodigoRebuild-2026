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
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;
public class IntakeFloor extends SubsystemBase{
    
    private final SparkMax IntakeMotor;
    private static final double VELOCIDADE_MAX = 0.25;
    private boolean IntakeGirando = false;

    public IntakeFloor(){
        IntakeMotor = new SparkMax(Constants.IntakeFloorMotor.IntakeMotor, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(60);

        IntakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);        
    }
    public void IntakeVelocidade(double velocidade){
    double velocidadeLimitada =
        MathUtil.clamp(velocidade, -VELOCIDADE_MAX, VELOCIDADE_MAX);

    IntakeMotor.set(velocidadeLimitada);

    IntakeGirando = Math.abs(velocidadeLimitada) > 0.02;
}
    public void ParaIntake(){
        IntakeMotor.set(0);
        IntakeGirando = false;
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake girando", IntakeGirando );
    }
}
