package OceanCrashLinearOpMode;

import android.graphics.Bitmap;
import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class Vision extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        throw new UnsupportedOperationException();
    }

    private final VuforiaLocalizer vuforia;
    private final LinearOpMode opMode;
    private final int park = 3;
    private final int highY = 400;
    private final int lowY = 270;
    private final int highXLeft = 300;
    private final int lowXLeft = 200;

    private final int highXRight = 500;
    private final int lowXRight = 400;

    private final int highX = 380;
    private final int lowX = 330;

    public Vision(LinearOpMode opMode){
        this.opMode = opMode;
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AQvLCbX/////AAABmTGnnsC2rUXvp1TAuiOSac0ZMvc3GKI93tFoRn4jPzB3uSMiwj75PNfUU6MaVsNZWczJYOep8LvDeM/3hf1+zO/3w31n1qJTtB2VHle8+MHWNVbNzXKLqfGSdvXK/wYAanXG2PBSKpgO1Fv5Yg27eZfIR7QOh7+J1zT1iKW/VmlsVSSaAzUSzYpfLufQDdE2wWQYrs8ObLq2kC37CeUlJ786gywyHts3Mv12fWCSdTH5oclkaEXsVC/8LxD1m+gpbRc2KC0BXnlwqwA2VqPSFU91vD8eCcD6t2WDbn0oJas31PcooBYWM6UgGm9I2plWazlIok72QG/kOYDh4yXOT4YXp1eYh864e8B7mhM3VclQ";
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();
    }

    public Bitmap getImage() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        return bm;
    }

    public int getParkLeft() throws InterruptedException {
        Bitmap rgbImage = getImage();
        int r = 0, g = 0, b = 0;

        for (int y = lowY; y < highY; y++) {
            for (int x = lowXLeft; x < highXLeft; x++) {
                int pixel = rgbImage.getPixel(x, y);
                r += red(pixel);
                b += blue(pixel);
                g += green(pixel);
            }
        }

        if (r > g && r > b) {
            return 1;
        }
        else if (b > r && b > g) {
            return 3;
        }
        else {
            return 2;
        }
    }

    public int getPark() throws InterruptedException {
        Bitmap rgbImage = getImage();
        int r = 0, g = 0, b = 0;

        for (int y = lowY; y < highY; y++) {
            for (int x = lowX; x < highX; x++) {
                int pixel = rgbImage.getPixel(x, y);
                r += red(pixel);
                b += blue(pixel);
                g += green(pixel);
            }
        }

        if (r > g && r > b) {
            return 1;
        }
        else if (b > r && b > g) {
            return 3;
        }
        else {
            return 2;
        }
    }

    public int getParkRight() throws InterruptedException {
        Bitmap rgbImage = getImage();
        int r = 0, g = 0, b = 0;

        for (int y = lowY; y < highY; y++) {
            for (int x = lowXRight; x < highXRight; x++) {
                int pixel = rgbImage.getPixel(x, y);
                r += red(pixel);
                b += blue(pixel);
                g += green(pixel);
            }
        }

        if (r > g && r > b) {
            return 1;
        }
        else if (b > r && b > g) {
            return 3;
        }
        else {
            return 2;
        }
    }
}
