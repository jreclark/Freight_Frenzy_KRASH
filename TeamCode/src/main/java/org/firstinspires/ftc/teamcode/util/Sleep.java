package org.firstinspires.ftc.teamcode.util;

public class Sleep {
    public Sleep() {
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
