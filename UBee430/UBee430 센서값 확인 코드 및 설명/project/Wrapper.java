public class Wrapper {
//static block
	static {
		System.loadLibrary("test_serial_jni"); //libgreeting.so
	}
//Native function declaration.
	public native void print();
}
