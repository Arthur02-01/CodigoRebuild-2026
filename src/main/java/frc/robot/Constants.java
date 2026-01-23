package frc.robot;

import frc.robot.subsystems.IntakeFloor;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Shooter{
   public static final int ShooterArlindo =10;
     public static final int ShooterBoquinha =11;
  }
  public static class EncoderShooter{
    public static final int ArlindoEncoder = 13;
    public static final int BoquinhaEncoder = 12;
  }
  public static final class TractionConstants {

   // ===== SPARK MAX - TRAÇÃO =====
  // Lado direito
  public static final int rightFrontMotorID = 1;
  public static final int rightBackMotorID  = 2;

  // Lado esquerdo
  public static final int leftFrontMotorID  = 3;
  public static final int leftBackMotorID   = 4;

  // ===== ENCODERS (PORTAS DIO DO roboRIO) =====
  // Ajuste se os fios estiverem em outras portas
  public static final int leftEncoderChannelA  = 32;
  public static final int rightEncoderChannelA = 4;
    }
    public static class Alinhador{
      public static final int AlinhadorMotor =9;
    }
    public static class FFAlinhador{
      public static final double kS = 0.15;
      public static final double kG = 0.45;
      public static final double kV = 0.0;
    }
    public static class EncoderAlinhador{
      public static final int AnguladorEncoder = 18;
    }

  public static final class IntakeFloorMotor{
    public static final int IntakeMotor = 21;
  }
}
