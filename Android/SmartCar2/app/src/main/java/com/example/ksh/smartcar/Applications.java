package com.example.ksh.smartcar;

import android.app.Application;
import android.content.Context;
import android.widget.ListView;

import com.example.ksh.smartcar.fragment.AutoFragment;
import com.example.ksh.smartcar.list.ListItem;
import com.example.ksh.smartcar.list.ListItemAdapter;
import com.example.ksh.smartcar.thread.SocketThread;

import java.util.ArrayList;

/**
 * Created by KSH on 2016-10-12.
 */

public class Applications extends Application {
    private SocketThread socketThread = null;
    private static ArrayList<ListItem> listItems = new ArrayList<ListItem>();
    private ListItemAdapter listItemAdapter = null;
    private ListView listView = null;
    private AutoFragment autoFragment = null;

    public SocketThread getSocketThread() {
        return socketThread;
    }

    public void setSocketThread(SocketThread socketThread) {
        this.socketThread = socketThread;
    }

    public ArrayList<ListItem> getListItems() {
        return listItems;
    }

    public void setListItems(ArrayList<ListItem> listItems) {
        this.listItems = listItems;
    }

    public ListItemAdapter getListItemAdapter() {
        return listItemAdapter;
    }

    public void setListItemAdapter(ListItemAdapter listItemAdapter) {
        this.listItemAdapter = listItemAdapter;
    }

    public ListView getListView() {
        return listView;
    }

    public void setListView(ListView listView) {
        this.listView = listView;
    }

    public AutoFragment getAutoFragment() {
        return autoFragment;
    }

    public void setAutoFragment(AutoFragment autoFragment) {
        this.autoFragment = autoFragment;
    }
}
