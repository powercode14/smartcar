package com.example.ksh.smartcar;

import android.app.FragmentTransaction;
import android.content.Context;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.example.ksh.smartcar.fragment.AutoFragment;
import com.example.ksh.smartcar.fragment.ConnectFragment;
import com.example.ksh.smartcar.fragment.ManualFragment;
import com.example.ksh.smartcar.fragment.MenuFragment;
import com.example.ksh.smartcar.fragment.VideoFragment;
import com.example.ksh.smartcar.interfaces.FragmentReplaceable;
import com.example.ksh.smartcar.interfaces.LayoutVisible;
import com.example.ksh.smartcar.list.ListItem;
import com.example.ksh.smartcar.list.ListItemAdapter;
import com.example.ksh.smartcar.thread.SocketThread;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity
        implements FragmentReplaceable, LayoutVisible, ConnectFragment.OnFragmentInteractionListener,
        ManualFragment.OnFragmentInteractionListener, AutoFragment.OnFragmentInteractionListener {
    public RelativeLayout container, container2;
    private TextView textViewStatus;
    private Logger logger;
    private ConnectFragment connectFragment;
    private MenuFragment menuFragment;
    private ManualFragment manualFragment;
    private VideoFragment videoFragment;
    private AutoFragment autoFragment;
    private ListItemAdapter adapter;
    public InputMethodManager imm;
    Applications applications;
    private static String serverIP = "192.168.43.8";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        container   = (RelativeLayout) findViewById(R.id.container);
        container2  = (RelativeLayout) findViewById(R.id.container2);
        textViewStatus      = (TextView) findViewById(R.id.textViewStatus);
        logger              = new Logger(textViewStatus);
        connectFragment     = new ConnectFragment();
        menuFragment        = new MenuFragment();
        manualFragment      = new ManualFragment();
        videoFragment       = new VideoFragment();
        autoFragment        = new AutoFragment();

        getFragmentManager().beginTransaction().add(R.id.container, connectFragment);
        getFragmentManager().beginTransaction().add(R.id.container2, menuFragment);
        getFragmentManager().beginTransaction().hide(menuFragment).commit();
        container2.setVisibility(View.GONE);
        imm = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
        applications = (Applications) getApplicationContext();
        applications.setAutoFragment(autoFragment);
    }

    @Override
    public void replaceFragment(int index) {
        FragmentTransaction ft = getFragmentManager().beginTransaction();
        ft.replace(R.id.container, videoFragment);
        if(index == 1){
            ft.replace(R.id.container2, manualFragment);
            ft.setTransition(FragmentTransaction.TRANSIT_FRAGMENT_OPEN);
        } else if(index == 2){
            ft.replace(R.id.container2, autoFragment);
            ft.setTransition(FragmentTransaction.TRANSIT_FRAGMENT_OPEN);
        }
        ft.addToBackStack(null);
        ft.commit();
    }

    @Override
    public void visibleChange(int visibility) {
        container2.setVisibility(visibility);
    }

    @Override
    public void log(String log) {
        logger.log(log);
    }

    @Override
    public void updateList() {
        autoFragment.updateList();
    }

    public static String getServerIP() {
        return serverIP;
    }

    public static void setServerIP(String serverIP) {
        MainActivity.serverIP = serverIP;
    }
}
