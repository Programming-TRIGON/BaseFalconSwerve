package frc.robot;

import com.ctre.phoenix.ErrorCode;

import java.util.function.Supplier;

public class Util {
    //recursive function to check if the error code from the errorCodeSupplier is OK. If not, print to the console, and try again, up to attempts times.
    public static void checkError(Supplier<ErrorCode> errorCodeSupplier, int attempts) {
        ErrorCode errorCode = errorCodeSupplier.get();
        if (errorCode != ErrorCode.OK) {
            if (attempts > 0) {
                System.out.println("Error: " + errorCode.toString() + ". Retrying...");
                checkError(errorCodeSupplier, attempts - 1);
            } else {
                System.out.println("Error: " + errorCode.toString() + ". Giving up.");
            }
        }
    }



}
