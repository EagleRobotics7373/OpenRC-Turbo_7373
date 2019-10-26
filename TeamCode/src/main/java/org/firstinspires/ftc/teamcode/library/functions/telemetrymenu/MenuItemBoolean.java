package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu;

import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItem;

public class MenuItemBoolean extends MenuItem<Boolean> {
    private String key;
    private String description;
    private boolean value;

    public MenuItemBoolean(String key, String description, boolean startingValue) {
        this.key = key;
        this.description = description;
        this.value = startingValue;
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
    public Boolean getValue() {
        return value;
    }

    @Override
    boolean setValue(Boolean newValue) {
        this.value = newValue;
        return true;
    }

    @Override
    boolean iterateForward() {
        if (!value) {
            value = true;
            return true;
        } else return false;
    }

    @Override
    boolean iterateBackwards() {
        if (value) {
            value = false;
            return true;
        } else return false;
    }

    @Override
    boolean hasNext() {
        if (!value) return true;
        else return false;
    }

    @Override
    boolean hasPrevious() {
        if (value) return true;
        else return false;
    }
}
