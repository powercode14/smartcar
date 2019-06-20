package com.example.ksh.smartcar.list;

import android.content.Context;

/**
 * Created by KSH on 2016-10-15.
 */

public class ListItem {
    String id, temp, humi, lux;

    public ListItem(String id, String temp, String humi, String lux) {
        this.id = id;
        this.temp = temp;
        this.humi = humi;
        this.lux = lux;
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public String getTemp() {
        return temp;
    }

    public void setTemp(String temp) {
        this.temp = temp;
    }

    public String getHumi() {
        return humi;
    }

    public void setHumi(String humi) {
        this.humi = humi;
    }

    public String getLux() {
        return lux;
    }

    public void setLux(String lux) {
        this.lux = lux;
    }
}
