package com.example.ksh.smartcar.fragment;

import android.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;

import com.example.ksh.smartcar.R;
import com.example.ksh.smartcar.Applications;
import com.example.ksh.smartcar.interfaces.FragmentReplaceable;
import com.example.ksh.smartcar.thread.SocketThread;

public class ManualFragment extends Fragment implements View.OnClickListener{
    private Button buttonUp;
    private Button buttonLeftTurn;
    private Button buttonRightTurn;
    private Button buttonDown;
    private Button buttonStop;
    private Button buttonSense;
    private Button buttonList;
    private Applications applications;
    private SocketThread socketThread;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootview = inflater.inflate(R.layout.activity_manual, container, false);
        buttonUp = (Button) rootview.findViewById(R.id.buttonUp);
        buttonLeftTurn = (Button) rootview.findViewById(R.id.buttonLeftTurn);
        buttonRightTurn = (Button) rootview.findViewById(R.id.buttonRightTurn);
        buttonDown = (Button) rootview.findViewById(R.id.buttonDown);
        buttonStop = (Button) rootview.findViewById(R.id.buttonStop);
        buttonSense = (Button) rootview.findViewById(R.id.buttonSense);
        buttonList = (Button) rootview.findViewById(R.id.buttonList);

        buttonUp.setOnClickListener(this);
        buttonLeftTurn.setOnClickListener(this);
        buttonRightTurn.setOnClickListener(this);
        buttonDown.setOnClickListener(this);
        buttonStop.setOnClickListener(this);
        buttonSense.setOnClickListener(this);
        buttonList.setOnClickListener(this);

        applications = (Applications) getActivity().getApplicationContext();
        socketThread = applications.getSocketThread();
        return rootview;
    }

    @Override
    public void onClick(View v) {
        if(v == buttonUp || v == buttonLeftTurn || v == buttonRightTurn || v == buttonDown || v == buttonStop)
        {
            String sndOpkey = "";
            if(v == buttonUp)	        sndOpkey = "Up";
            if(v == buttonLeftTurn)     sndOpkey = "LeftTurn";
            if(v == buttonRightTurn)	sndOpkey = "RightTurn";
            if(v == buttonDown)	        sndOpkey = "Down";
            if(v == buttonStop)	        sndOpkey = "Stop";
            socketThread.send(sndOpkey); //소켓쓰레드의 send 메소드가 실행되며 각 버튼별 메시지가 전송된다.d
        } else if(v == buttonList){
            ((FragmentReplaceable)getActivity()).replaceFragment(2);
        } else if(v == buttonSense){
            Toast.makeText(getActivity(), "Sensing Start", Toast.LENGTH_SHORT).show();
            socketThread.send("Sense");
        }
    }

    public interface OnFragmentInteractionListener {
        void log(String log);
    }
}
