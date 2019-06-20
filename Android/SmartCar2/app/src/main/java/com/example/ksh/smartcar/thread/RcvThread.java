package com.example.ksh.smartcar.thread;

/*

import com.example.ksh.smartcar.Logger;
import com.example.ksh.smartcar.MainActivity;
import com.example.ksh.smartcar.list.ListItem;

import java.io.IOException;
import java.net.Socket;

*/
/**
 * Created by KSH on 2016-09-08.
 *//*


public class RcvThread implements Runnable {
    private Logger logger;
    private final int sizeBuf = 50;
    private Socket socket;
    private int flag = 1;
    private String rcvData = "Error";
    private byte[] rcvBuf = new byte[sizeBuf];
    private int rcvBufSize;
    private ListItem listItem;

    public RcvThread(Logger logger, Socket socket){
        this.logger = logger;
        this.socket = socket;
    }

    @Override
    public void run() {
        while(flag==1){ //어플이 실행되는동안 계속 서버에서 메시지를 받아야하기 때문에 무한루프를 돌고있어야 한다.
            try {
                rcvBufSize = socket.getInputStream().read(rcvBuf); //서버에서 메시지를 보내기 전까지 기다리다가 메시지가 도착하면 읽는다.
                rcvData = new String(rcvBuf, 0, rcvBufSize, "UTF-8"); //받은메시지를 String으로 저장한다
                if(rcvData.compareTo("[close]")==0) { //받은문자가 '[close]'이면 flag가 0이되므로 무한루프를 빠져나가게 된다.
                    flag = 0;
                } else if(rcvData.contains(":")) {
                    String id = rcvData.substring(0, rcvData.indexOf(":"));
                    String temp = rcvData.substring(rcvData.indexOf(":")+1, rcvData.indexOf(":", rcvData.indexOf(":")+1));
                    String humi = rcvData.substring(rcvData.indexOf(":", rcvData.indexOf(":")+1)+1, rcvData.indexOf(":", rcvData.indexOf(":", rcvData.indexOf(":")+1)+1));
                    String lux = rcvData.substring(rcvData.indexOf(":", rcvData.indexOf(":", rcvData.indexOf(":", rcvData.indexOf(":")+1)+1)+1));
                    //lister.addListItem(new ListItem(id, temp, humi, lux));
                }
                (MainActivity)
                logger.log("Receive Data : " + rcvData);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        logger.log("Exit loop");
    }

    public interface OnSocketThreadInteractionListener{
        void log(String log);
    }
}*/
