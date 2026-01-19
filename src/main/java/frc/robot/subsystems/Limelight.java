package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    /* ===== NETWORKTABLE ===== */

    private final NetworkTable table;

    /* ===== CONSTRUTOR ===== */

    public Limelight() {
        table = NetworkTableInstance
                .getDefault()
                .getTable("limelight");
    }

    /* ===== LEITURA DE DADOS ===== */

    /**
     * @return true se a Limelight estiver vendo um alvo válido
     */
    public boolean temAlvo() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * @return erro horizontal em graus (centro da imagem = 0)
     */
    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return ID da AprilTag detectada, -1 se nenhuma
     */
    public int getAprilTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    /* ===== CONTROLE DA LIMELIGHT ===== */

    /**
     * Liga os LEDs da Limelight
     */
    public void ligarLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    /**
     * Desliga os LEDs da Limelight
     */
    public void desligarLED() {
        table.getEntry("ledMode").setNumber(1);
    }

    /**
     * Seleciona o pipeline configurado na Limelight
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
}
