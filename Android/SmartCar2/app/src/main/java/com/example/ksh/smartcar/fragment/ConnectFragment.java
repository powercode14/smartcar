package com.example.ksh.smartcar.fragment;

import android.app.Fragment;
import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.ksh.smartcar.MainActivity;
import com.example.ksh.smartcar.interfaces.LayoutVisible;
import com.example.ksh.smartcar.list.ListItem;
import com.example.ksh.smartcar.Logger;
import com.example.ksh.smartcar.R;
import com.example.ksh.smartcar.thread.SocketThread;

import java.util.ArrayList;

public class ConnectFragment extends Fragment implements View.OnClickListener, SocketThread.OnSocketThreadInteractionListener {
    private OnFragmentInteractionListener mListener;
    private EditText editTextIPAddress;
    private Button buttonConnect;
    private Button buttonClose;
    private ListView listView;
    private Logger logger;
    private ArrayList<ListItem> listItems;

    private SocketThread socketThread = null;
    private Thread sThread;
    private ManualFragment manualFragment;
    private InputMethodManager imm;
    private boolean isSocketThread = false;

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        if (context instanceof OnFragmentInteractionListener) {
            mListener = (OnFragmentInteractionListener) context;
        } else {
            throw new RuntimeException(context.toString() + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.activity_connect, container, false);
        editTextIPAddress   = (EditText)rootView.findViewById(R.id.editTextIPAddress);
        buttonConnect       = (Button)rootView.findViewById(R.id.buttonConnect);
        buttonClose         = (Button)rootView.findViewById(R.id.buttonClose);
        listView            = (ListView) rootView.findViewById(R.id.listView);
        editTextIPAddress.setText(MainActivity.getServerIP());
        buttonConnect.setOnClickListener(this);
        buttonClose.setOnClickListener(this);

        manualFragment = new ManualFragment();
        imm = (InputMethodManager)getActivity().getSystemService(Context.INPUT_METHOD_SERVICE);

        return rootView;
    }

    @Override
    public void onClick(View arg0) {
        if(arg0 == buttonConnect)// connect버튼 클릭하면
        {
            imm.hideSoftInputFromWindow(editTextIPAddress.getWindowToken(), 0); //키패드 숨기기
            createSocketThread();
            ((LayoutVisible)getActivity()).visibleChange(View.VISIBLE);
        }

        if(arg0 == buttonClose) //close버튼을 클릭하면
        {
            imm.hideSoftInputFromWindow(editTextIPAddress.getWindowToken(), 0);
            if(socketThread != null && socketThread.isConnected()) {
                socketThread.buttonClose(); //소켓쓰레드의 buttonClose메소드가 실행된다.
            } else {
                log("Already disconnected");
            }
            ((LayoutVisible)getActivity()).visibleChange(View.GONE);
        }
    }

    public void createSocketThread(){
        MainActivity.setServerIP(editTextIPAddress.getText().toString()); //텍스트에있는 ip주소를 변수 server에 저장
        socketThread = new SocketThread(MainActivity.getServerIP(), getActivity()); //소켓은 메인액티비티에서 생성할수 없으므로 소켓쓰레드를 만든다.
        sThread = new Thread(socketThread); //소켓쓰레드는 하위클래스이므로 상위클래스인 Thread클래스에서 다시 생성함.
        sThread.start(); //쓰레드의 run메소드를 실행시키기위한 문장
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    @Override
    public void log(String log) {
        ((MainActivity)getActivity()).log(log);
    }

    public interface OnFragmentInteractionListener {
        void log(String log);
    }
}