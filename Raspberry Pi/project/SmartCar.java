import java.io.IOException;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStreamReader;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketAddress;

import java.util.Enumeration;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;

public class SmartCar {
	public static void main(String[] args) {
        new Thread(new SocketThread()).start();
	}
}

class SocketThread implements Runnable{
	private ServerSocket serverSocket;
	private int port = 8888;
    private Socket clientSocket;

    public void run(){
        try{
            init();
            while(true){
                clientSocket = serverSocket.accept();
		        new Thread(new RcvThread(clientSocket, clientSocket.getRemoteSocketAddress())).start(); //서버에서도 클라이언트와 마찬가지로 받기전용 쓰레드를 생성한다.
		        if(clientSocket.isConnected()) { //클라이언트가 접속되었으면 클라이언트의 IP를 출력함
		            System.out.println("Connected Client IP : " + clientSocket.getRemoteSocketAddress());
		        }
            }
		} catch (Exception e) {
			e.printStackTrace();
		}
    }

    public void init(){
        try{
		    serverSocket = new ServerSocket(port); //서버소켓 생성
		    System.out.println("Start Teleop Server : " + getIpAddress() + "(" + port + ")"); //서버가 시작되면 서버의 ip주소와 port번호 출력
        } catch(Exception e){
            System.out.println("Can't open the port.");
            System.exit(-1);
        }
    }

	private String getIpAddress(){
		String ip = null;
		try{
			boolean isLoopBack=true;
			Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces();
			while(en.hasMoreElements()){
				NetworkInterface ni = en.nextElement();
				if(ni.isLoopback())
					continue;
				Enumeration<InetAddress> inetAddresses = ni.getInetAddresses();
				while(inetAddresses.hasMoreElements()){
					InetAddress ia = inetAddresses.nextElement();
					if(ia.getHostAddress() != null && ia.getHostAddress().indexOf(".") != -1){
						ip = ia.getHostAddress();
						isLoopBack = false;
						break;
					}
				}
				if(!isLoopBack)
					break;
			}
		} catch (SocketException e){
			e.printStackTrace();
		}
		return ip;
	}
}

class SensorThread implements Runnable{
    private Socket clientSocket;
    public String strSensor;
    private Sensor sensor;

    public SensorThread(Socket clientSocket){
        this.clientSocket = clientSocket;
    }

    @Override
    public void run(){
        try{
            PrintWriter pw = new PrintWriter(clientSocket.getOutputStream());
            sensor = new Sensor();
            while(true) {
                while(!sensor.sense().equals(strSensor)){
                    strSensor = sensor.sense();
                    System.out.println(strSensor);
                }
                pw.println(strSensor);
                pw.flush();
                Thread.sleep(3000);
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace(); 
        }
    }
    /*class SensorTest{
        synchronized public String sense(){
            return "1:26:40:74";
        }
    }*/
}

class RcvThread implements Runnable {
	private Socket clientSocket;
	private SocketAddress clientAddress;
    private BufferedReader br;
	private PrintWriter pw;
    private String rcvData;
    private Thread sensorThread;

    final GpioController gpio = GpioFactory.getInstance();

    //final GpioPinDigitalOutput pinlpwm = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_04, "lpwm", PinState.LOW);
    final GpioPinDigitalOutput pinlfwd = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_05, "lfwd", PinState.LOW);
    final GpioPinDigitalOutput pinlrev = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_06, "lrev", PinState.LOW);

    //final GpioPinDigitalOutput pinrpwm = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_00, "rpwm", PinState.LOW);
    final GpioPinDigitalOutput pinrfwd = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02, "rfwd", PinState.LOW);
    final GpioPinDigitalOutput pinrrev = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03, "rrev", PinState.LOW);



	public RcvThread(Socket clientSocket, SocketAddress clientAddress) {
		this.clientSocket = clientSocket;
		this.clientAddress = clientAddress;
	}

	@Override
	public void run() {
		try{
			br = new BufferedReader(new InputStreamReader(clientSocket.getInputStream())); //받을메시지 저장소
			pw = new PrintWriter(clientSocket.getOutputStream()); //보낼메시지 저장소
			
            sensorThread = new Thread(new SensorThread(clientSocket));
            sensorThread.start();
            while(true){ //클라이언트에서 메시지가 도착할 때 까지 대기하다가 메시지를 받으면 루프문장 수행
                rcvData = br.readLine();
                if(rcvData.contains("Close")){
                    pw.println(rcvData);
                    pw.flush();
                    break;
                } else {
				    if(rcvData.contains("Up")){ //Up메시지를 받았으면 Go!를 출력함. 나머지는 마찬가지
				        //pinlpwm.high();
                        pinlfwd.high();
                        pinlrev.low();
                        //pinrpwm.high();
                        pinrfwd.high();
                        pinrrev.low();
                        System.out.println("Go!");
                    }
    			    if(rcvData.contains("LeftTurn")){
                        //pinlpwm.high();
                        pinlfwd.high();
                        pinlrev.low();
                        //pinrpwm.high();
                        pinrfwd.low();
                        pinrrev.high();
	    			    System.out.println("LeftTurn!");
                    }
    			    if(rcvData.contains("RightTurn")){
                        //pinlpwm.high();
                        pinlfwd.low();
                        pinlrev.high();
                        //pinrpwm.high();
                        pinrfwd.high();
                        pinrrev.low();
    				    System.out.println("RighTurn!");
                    }
    			    if(rcvData.contains("Down")){
                        //pinlpwm.high();
                        pinlfwd.low();
                        pinlrev.high();
                        //pinrpwm.high();
                        pinrfwd.low();
                        pinrrev.high();
    				    System.out.println("Back!");
                    }
    			    if(rcvData.contains("Stop")){
                        pinlfwd.low();
                        pinlrev.low();
                        pinrfwd.low();
                        pinrrev.low();

    			        System.out.println("Stop!");
                    }
				    System.out.println("Received data : " + rcvData + " (" + clientAddress + ")"); //받은데이터와 클라이언트 IP를 한 번 출력해줌
                    pw.println(rcvData);
                    pw.flush();
                }
			}
			System.out.println(clientSocket.getRemoteSocketAddress() + " Closed");
		} catch(IOException e) {
			e.printStackTrace();
		} finally {
            try {
                sensorThread.interrupt();
                clientSocket.close();
                System.out.println("Disconnected! Client IP : " + clientAddress);
            } catch (Exception e) {
                e.printStackTrace();
            }
		}
	}
}
