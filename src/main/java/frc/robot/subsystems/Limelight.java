package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /* ===== LEITURAS ===== */

    public boolean temAlvo() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTa() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public int getAprilTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    /* ===== CONTROLES ===== */

    public void ligarLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void desligarLED() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
}
