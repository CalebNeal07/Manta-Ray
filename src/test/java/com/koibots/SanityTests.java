// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots;

import static org.junit.jupiter.api.Assertions.*;

import com.koibots.robot.Constants;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ExampleTest {
    @Test
    void dontRepeatCANIDs() {
        Field[] declaredFields = Constants.CANDeviceIDs.class.getDeclaredFields();
        List<Integer> ids = new ArrayList<>();
        for (Field field : declaredFields) {
            if (java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                try {
                    int id = field.getInt(new Constants.CANDeviceIDs());
                    if (ids.contains(id)) {
                        System.out.println("Duplicated CAN ID at " + field.getName());
                        fail();
                    }
                    ids.add(id);
                } catch (IllegalAccessException e) {
                    System.out.println("Failed to parse " + field.getName());
                }
            }
        }
    }
}
