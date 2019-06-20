package com.example.ksh.smartcar.list;

import android.content.Context;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.widget.TableLayout;
import android.widget.TextView;

import com.example.ksh.smartcar.R;

/**
 * Created by KSH on 2016-10-15.
 */

public class ListItemView extends TableLayout {
    TextView idTextView, tempTextView, humiTextView, luxTextView;

    public ListItemView(Context context) {
        super(context);
        init(context);
    }

    public ListItemView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    private void init(Context context){
        LayoutInflater inflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        inflater.inflate(R.layout.list_item, this, true);
        idTextView = (TextView) findViewById(R.id.list_item1);
        tempTextView = (TextView) findViewById(R.id.list_item2);
        humiTextView = (TextView) findViewById(R.id.list_item3);
        luxTextView = (TextView) findViewById(R.id.list_item4);
    }

    public void setIdTextView(String id) {
        idTextView.setText(id);
    }

    public void setTempTextView(String temp) {
        tempTextView.setText(temp);
    }

    public void setHumiTextView(String humi) {
        humiTextView.setText(humi);
    }

    public void setLuxTextView(String lux) {
        luxTextView.setText(lux);
    }
}
