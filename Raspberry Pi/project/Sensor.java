import java.lang.reflect.Field;

public class Sensor {
//static block
     static {
         //System.loadLibrary("sensor"); //lib라이브러리파일명.so
         try {
             setLibraryPath("/home/pi/project/sensor");
         } catch (Exception e) {
             e.printStackTrace();
         }
         try {
             System.loadLibrary("sensor");
         } catch (UnsatisfiedLinkError e) {
             System.err.println ("The dynamic link library for Java could not be"
                     + "loaded. \nConsider using\njava -Djava.library.path =\n" + e.getMessage());
             throw e;
         } catch (Exception e) {
             e.printStackTrace();
         }
     }
 //Native function declaration.
     public native String sense();
     public static void setLibraryPath(String path) throws Exception {
         System.setProperty("java.library.path", path);
         final Field sysPathsField = ClassLoader.class.getDeclaredField("sys_paths");
         sysPathsField.setAccessible(true);
         sysPathsField.set(null, null);
     }
}
