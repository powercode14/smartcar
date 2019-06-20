package com.example.ksh.smartcar.list;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import com.example.ksh.smartcar.R;

import java.util.ArrayList;

/**
 * Created by KSH on 2016-10-21.
 */

public class ListItemAdapter extends ArrayAdapter<ListItem> {
    private LayoutInflater inflater;
    private Context context;
    private int layout;
    int count = 0;
    ArrayList<ListItem> items;

    public ListItemAdapter(Context context, int layout, ArrayList<ListItem> items) {
        super(context, 0, items);
        this.context = context;
        this.layout = layout;
        this.items = items;
        inflater = LayoutInflater.from(this.context);
        count = items.size();
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        View view;
        if(convertView == null) {
            view = inflater.inflate(R.layout.list_item, null);
        } else {
            view = convertView;
        }
        ListItem item = this.getItem(position);

        if (item != null) {
            TextView ti1 = (TextView) view.findViewById(R.id.list_item1);
            TextView ti2 = (TextView) view.findViewById(R.id.list_item2);
            TextView ti3 = (TextView) view.findViewById(R.id.list_item3);
            TextView ti4 = (TextView) view.findViewById(R.id.list_item4);
            ti1.setText(item.getId());
            ti2.setText(item.getTemp());
            ti3.setText(item.getHumi());
            ti4.setText(item.getLux());
        }

        return view;
    }

    @Override
    public int getCount() {
        return count;
    }

    public void addItem(ListItem item){
        super.add(item);
    }
}
