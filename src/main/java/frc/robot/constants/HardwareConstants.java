package frc.robot.constants;

public class HardwareConstants {
    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANivore");

        public final String busName;
        CANBus(final String busName) {
            this.busName = busName;
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(busName);
        }
    }
}
