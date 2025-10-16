package frc.lib;

import com.ctre.phoenix6.StatusCode;

import java.util.function.Supplier;

import static frc.lib.Util.asyncExecutor;

public class PhoenixUtil {
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    public static void tryUntilOkAsync(int maxAttempts, Supplier<StatusCode> command) {
        asyncExecutor.execute(() -> tryUntilOk(maxAttempts, command));
    }
}
