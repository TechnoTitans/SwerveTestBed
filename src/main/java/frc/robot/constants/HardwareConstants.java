package frc.robot.constants;

public class HardwareConstants {
    public enum CANBus {
        RIO("rio"),
        CANIVORE("CANIVORE");

        public final String name;
        CANBus(final String name) {
            this.name = name;
        }

        public com.ctre.phoenix6.CANBus toPhoenix6CANBus() {
            return new com.ctre.phoenix6.CANBus(name);
        }
    }
}
