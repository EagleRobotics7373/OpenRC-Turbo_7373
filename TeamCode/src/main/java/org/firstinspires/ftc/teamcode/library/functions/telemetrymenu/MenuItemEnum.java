package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu;

import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItem;

import java.util.ArrayList;
import java.util.Arrays;

public class MenuItemEnum<T> extends MenuItem<T> {
    private String key;
    private String description;
    private ArrayList<T> allItems;
    private int currentItemPosition = 0;

    public MenuItemEnum(String key, String description, T startingElement, T ... otherAllowedElements) {
        this.key = key;
        this.description = description;
        allItems = new ArrayList<>();
        allItems.add(startingElement);
        allItems.addAll(Arrays.asList(otherAllowedElements));
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
    public T getValue() {
        return allItems.get(currentItemPosition);
    }

    @Override
    boolean setValue(T newValue) {
        int newItemPosition = allItems.indexOf(newValue);
        if (newItemPosition != -1) {
            currentItemPosition = newItemPosition;
            return true;
        } else return false;
    }

    @Override
    boolean iterateForward() {
        if (hasNext()) {
            currentItemPosition++;
            return true;
        } else return false;
    }

    @Override
    boolean iterateBackwards() {
        if (hasPrevious()) {
            currentItemPosition--;
            return true;
        } else return false;
    }

    @Override
    boolean hasNext() {
        if (currentItemPosition < allItems.size()-1) return true;
        else return false;
    }

    @Override
    boolean hasPrevious() {
        if (currentItemPosition > 0) return true;
        else return false;
    }
}
