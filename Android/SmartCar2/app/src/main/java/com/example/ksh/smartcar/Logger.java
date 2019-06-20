package com.example.ksh.smartcar;

import android.os.Handler;
import android.widget.TextView;

/**
 * Created by KSH on 2016-09-08.
 */

class ps implements Runnable{
    TextView t;
    String s;
    public ps(TextView t, String s){
        this.t = t;
        this.s = s;
    }
    public void run(){
        t.setText(s);
    }
}

public class Logger {
    Handler h;
    TextView t;
    public Logger(TextView t){
        this.t = t;
        h = new Handler();
    }
    public void log(String s){
    h.post(new ps(t, s));
    }
}
