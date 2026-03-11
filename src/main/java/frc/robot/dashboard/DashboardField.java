package frc.robot.dashboard;

import java.util.function.BooleanSupplier;

public record DashboardField(String name, BooleanSupplier trigger) {
}
