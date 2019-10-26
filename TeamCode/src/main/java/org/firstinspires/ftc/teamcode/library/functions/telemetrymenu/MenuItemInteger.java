package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu;

import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItem;

public class MenuItemInteger extends MenuItem<Integer> {
    private String key;
    private String description;
    private int value;
    private int lowestPossible;
    private int highestPossible;

    public MenuItemInteger(String key, String description, int startingValue, int lowestPossible, int highestPossible) {
        this.key = key;
        this.description = description;
        this.lowestPossible = lowestPossible;
        this.highestPossible = highestPossible;

        if (startingValue <= highestPossible && startingValue >= lowestPossible)
            this.value = startingValue;
        else throw new IllegalArgumentException("startingValue of " + startingValue
                + " is not between lowestPossible of " +lowestPossible
                +" and highestPossible of " + highestPossible);
        }

    @Override
    String getKey() {
        return key;
    }

    @Override
    String getDescription() {
        return description;
    }

    @Override
    public Integer getValue() {
        return value;
    }

    @Override
    boolean setValue(Integer newValue) {
        if (newValue >= lowestPossible && newValue <= highestPossible) {
            this.value = newValue;
            return true;
        } else return false;
    }

    @Override
    boolean iterateForward() {
        if (hasNext()) {
            value++;
            return true;
        } else return false;
    }

    @Override
    boolean iterateBackwards() {
        if (hasPrevious()) {
            value--;
            return true;
        } else return false;
    }

    @Override
    boolean hasNext() {
        if (value < highestPossible) return true;
        else return false;
    }

    @Override
    boolean hasPrevious() {
        if (value > lowestPossible) return true;
        else return false;
    }
}
