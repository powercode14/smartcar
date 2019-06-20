package com.example.ksh.smartcar.thread;

import android.app.Activity;
import android.os.Handler;
import android.view.View;
import android.widget.ListView;
import android.widget.Toast;

import com.example.ksh.smartcar.Applications;
import com.example.ksh.smartcar.MainActivity;
import com.example.ksh.smartcar.fragment.AutoFragment;
import com.example.ksh.smartcar.interfaces.LayoutVisible;
import com.example.ksh.smartcar.list.ListItem;
import com.example.ksh.smartcar.list.ListItemAdapter;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;

import java.net.Socket;
import java.util.ArrayList;

public class SocketThread implements Runnable{
    private Socket socket;
    private int port = 8888;
    private String server;
    private BufferedReader reader;
    private PrintWriter writer;
    private boolean connected = false;
    private Handler handler;
    private Activity activity;
    private Thread rcvThread;
    private Applications applications;
    private ArrayList<ListItem> items;
    private ListItemAdapter adapter;
    private ListView listView;
    private Activity autoActivity;
    private ListItem item;

    public SocketThread(String server, Activity activity){ //소켓쓰레드의 생성자. serverip와 logger클래스를 초기화함
        this.server = server;
        this.activity = activity;
        applications = (Applications) activity.getApplicationContext();
        handler = new Handler();
    }

    public interface OnSocketThreadInteractionListener{
        void log(String log);
    }

    @Override
    public void run() { //메인액티비티에서 start메소드가 호출되면 실행되는 부분
        try {
            if (socket != null) { //소켓이 null이 아니면 소켓을 비운다.
                socket.close();
                socket = null;
            }
            socket = new Socket(server, port); //ip주소와 port번호를 가지고 소켓 생성
            if(socket.isConnected()) {

                ((MainActivity)activity).log("Connected");
                connected = true;
                handler.post(new Runnable() {
                    @Override
                    public void run() {
                        ((LayoutVisible) activity).visibleChange(View.VISIBLE);
                    }
                });

                reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                writer = new PrintWriter(socket.getOutputStream()); //서버로 메시지를 보내기위한 그릇?정도로 해석
                readerExecute();
                applications.setSocketThread(this);
            }
        } catch(IOException e){ //접속실패했을경우
            ((MainActivity)activity).log("Fail to connect");
            e.printStackTrace();
        }
    }

    public void readerExecute(){
        rcvThread = new Thread(new ReceiveThread());
        rcvThread.start();
    }

    class ReceiveThread extends Thread{
        BufferedReader rcvBuf;
        AutoFragment autoFragment = applications.getAutoFragment();

        @Override
        public void run() {

            while(true){
                try{
                    rcvBuf = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    final String rcvData = reader.readLine();

                    if("[close]".equals(rcvData)) {
                        break;
                    } else if(rcvData.contains(":")) {
                        items = applications.getListItems();

                        handler.post(new Runnable() {
                            @Override
                            public void run() {
                                String id = rcvData.substring(0, rcvData.indexOf(":"));
                                String temp = rcvData.substring(rcvData.indexOf(":") + 1, rcvData.indexOf(":", rcvData.indexOf(":") + 1));
                                String humi = rcvData.substring(rcvData.indexOf(":", rcvData.indexOf(":") + 1) + 1, rcvData.indexOf(":", rcvData.indexOf(":", rcvData.indexOf(":") + 1) + 1));
                                String lux = rcvData.substring(rcvData.indexOf(":", rcvData.indexOf(":", rcvData.indexOf(":") + 1) + 1) + 1);
                                ListItem item = new ListItem(id, temp, humi, lux);
                                items.add(item);
                                if(autoFragment.getmListener() != null){
                                    ((MainActivity)activity).updateList();
                                }
                                Toast.makeText(activity, "Data was collected from Area #"+ id, Toast.LENGTH_LONG).show();
                            }
                        });
                    } else {
                        ((MainActivity)activity).log(rcvData.toString());
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public boolean isConnected(){
        return connected;
    }

    public void buttonClose(){ //닫기버튼을 클릭했을 때 실행되는 메소드
        if(socket != null){
            try{
                String sndOpkey = "[close]"; //RcvThread가 무한루프에서 빠져나오려면 서버로 '[close]'를 보내고나서 다시 받아야한다.
                send(sndOpkey);
                rcvThread.join();
                socket.close(); //소켓닫기
                socket = null;
                connected = false;
                ((MainActivity)activity).log("Closed!"); //상태텍스트에 Closed 표시
            } catch (Exception e){
                ((MainActivity)activity).log("Fail to close");
                e.printStackTrace();
            }
        }
    }

    public void send(String sndOpkey) {
        writer.println(sndOpkey);
        writer.flush();
    }
}