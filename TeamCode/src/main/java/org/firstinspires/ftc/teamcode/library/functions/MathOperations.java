package org.firstinspires.ftc.teamcode.library.functions;
/**
 * Created by Kk4jr on 9/28/2017.
 */

public class MathOperations {
    /**
     *
     * @param numberToClip This is the number to be range clipped
     * @param lowerBound Lower Bound
     * @param upperBound Upper Bound
     */
    public static double rangeClip(double numberToClip, double lowerBound, double upperBound){
        if(numberToClip >= upperBound)
            return upperBound;
        if(numberToClip <= lowerBound)
            return lowerBound;
        return numberToClip;
    }

    public static double rangeClip(double numberToClip){
        if(numberToClip >= 1)
            return 1;
        if(numberToClip <= -1)
            return -1;
        return numberToClip;
    }

    public static float rangeClip(float numberToClip, float lowerBound, float upperBound){
        if(numberToClip >= upperBound)
            return upperBound;
        if(numberToClip <= lowerBound)
            return lowerBound;
        return numberToClip;
    }

    /**
     * Sets an input value to a specified default value if the input is within the range bounds
     * @param numberToBuffer Input value
     * @param lowerBound Lower bound for buffer cutoff
     * @param upperBound Upper bound for buffer cutoff
     * @param defaultValue If numberToBuffer is in between specified bounds, return this value
     * @return Input value (if outside of buffer range) or default value (if inside buffer range)
     */
    public static double rangeBuffer(double numberToBuffer, double lowerBound, double upperBound, double defaultValue) {
        if(numberToBuffer > lowerBound && numberToBuffer < upperBound) return defaultValue;
        else return numberToBuffer;
    }

    public static double rangeBuffer(double numberToBuffer, double lowerBound, double upperBound) {
        return rangeBuffer(numberToBuffer, lowerBound, upperBound, 0);
    }

    /**
     * Sets an input value to a specified default value if the input is within the range bounds
     * @param numberToBuffer Input value
     * @param lowerBound Lower bound for buffer cutoff
     * @param upperBound Upper bound for buffer cutoff
     * @param defaultValue If numberToBuffer is in between specified bounds, return this value
     * @return Input value (if outside of buffer range) or default value (if inside buffer range)
     */
    public static float rangeBuffer(float numberToBuffer, float lowerBound, float upperBound, float defaultValue) {
        if(numberToBuffer > lowerBound && numberToBuffer < upperBound) return defaultValue;
        else return numberToBuffer;
    }

    public static double rangeBuffer(float numberToBuffer, float lowerBound, float upperBound) {
        return rangeBuffer(numberToBuffer, lowerBound, upperBound, 0);
    }
}
