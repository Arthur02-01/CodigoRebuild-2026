package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Shooter{
   public static final int ShooterArlindo =8;
     public static final int ShooterBoquinha =9;
  }
  public static class EncoderShooter{
    public static final int ArlindoEncoder = 3;
    public static final int BoquinhaEncoder = 4;
  }
  public static final class TractionConstants {

   // ===== SPARK MAX - TRAÇÃO =====
  // Lado direito
  public static final int rightFrontMotorID = 5;
  public static final int rightBackMotorID  = 2;

  // Lado esquerdo
  public static final int leftFrontMotorID  = 3;
  public static final int leftBackMotorID   = 1;

  // ===== ENCODERS (PORTAS DIO DO roboRIO) =====
  // Ajuste se os fios estiverem em outras portas
  public static final int leftEncoderChannelA  = 0;
  public static final int leftEncoderChannelB  = 1;

  public static final int rightEncoderChannelA = 2;
  public static final int rightEncoderChannelB = 3;
    }
    public static class Alinhador{
      public static final int AlinhadorMotor =2;
    }
    public static class FFAlinhador{
      public static final double kS = 0.15;
      public static final double kG = 0.45;
      public static final double kV = 0.0;
    }
    public static class EncoderAlinhador{
      public static final int AlinhadorEncoder = 4;
    }
}
