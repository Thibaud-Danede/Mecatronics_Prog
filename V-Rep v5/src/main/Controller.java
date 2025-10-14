package main;

import coppelia.*;
import com.googlecode.javacv.cpp.opencv_core;

import javafx.fxml.FXML;
import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.image.PixelWriter;

import utils.Delay;
import utils.ImageViewer;
import utils.Timer;
import utils.Utils;

import java.awt.*;
import java.util.Arrays;
import java.awt.image.BufferedImage;
import java.util.Collections;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_highgui.cvLoadImage;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvMatchTemplate;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_TM_CCOEFF_NORMED;

/**
 * Created by Theo Theodoridis.
 * Class    : Controller
 * Version  : v1.0
 * Date     : © Copyright 01/11/2020
 * User     : ttheod
 * email    : ttheod@gmail.com
 * Comments : None.
 **/

public class Controller {
    @FXML
    private Button btnConnect;
    @FXML
    private Button btnRight;
    @FXML
    private Button btnLeft;
    @FXML
    private Button btnBack;
    @FXML
    private Button btnForward;
    @FXML
    private Button btnStop;
    @FXML
    private Canvas canvasCamera;
    @FXML
    PixelWriter pw;
    @FXML
    private Label lblSensor0;
    @FXML
    private Label lblSensor1;
    @FXML
    private Label lblSensor2;
    @FXML
    private Label lblSensor3;
    @FXML
    private Label lblSensor4;
    @FXML
    private Label lblSensor5;
    @FXML
    private Label lbl;
    @FXML
    private Label lblGpsX;
    @FXML
    private Label lblGpsY;
    @FXML
    private Label lblGpsZ;
    @FXML
    private Label lblRightWheel;
    @FXML
    private Label lblLeftWheel;

    private String defaultButtonStyle;

    /**
     * Updates:
     **/

    private Color imageCamera[][];
    private double gpsValues[] = new double[3];
    private double sonarValues[] = new double[6];
    private double encoderValues[] = new double[2];

    private boolean runGPS = true;
    private boolean runCamera = true;
    private boolean runMotion = true;
    private boolean runSensors = true;
    private boolean runWheelEncoder = true;

    /**
     * Robot:
     **/

    private IntW cameraHandle = new IntW(1);
    private IntW leftWheelHandle = new IntW(1);
    private IntW rightWheelHandle = new IntW(1);

    private boolean running = false;
    private boolean firstCameraRead = true;
    private boolean firstSensor0Read = true;
    private boolean firstSensor1Read = true;
    private boolean firstSensor2Read = true;
    private boolean firstSensor3Read = true;
    private boolean firstSensor4Read = true;
    private boolean firstSensor5Read = true;
    private boolean firstLeftWheelCall = true;
    private boolean firstRightWheelCall = true;

    private char dir = 's'; // Direction.
    private int vel = 5;    // Velocity.

    /**
     * Camera:
     **/

    private double targetMinScore = 0.0;
    private double targetMaxScore = 0.0;

    private int resolutionCamera = 256;
    private CvPoint point = new CvPoint();
    private IntWA resolution = new IntWA(1);
    private CharWA image = new CharWA(resolutionCamera * resolutionCamera * 3);
    private char imageArray[] = new char[resolutionCamera * resolutionCamera * 3];
    private Color colorMatrix[][] = new Color[resolutionCamera][resolutionCamera];
    private BufferedImage bufferedImage = new BufferedImage(resolutionCamera, resolutionCamera, BufferedImage.TYPE_INT_RGB);

    /**
     * Wheel encoders:
     **/

    private float dxRight = 0;
    private float totalRightJointPosition = 0;
    private float currentRightJointPosition = 0;
    private float previousRightJointPosition = 0;
    private FloatW robotRightJointPosition = new FloatW(3);

    private float dxLeft = 0;
    private float totalLeftJointPosition = 0;
    private float currentLeftJointPosition = 0;
    private float previousLeftJointPosition = 0;
    private FloatW robotLeftJointPosition = new FloatW(3);

    /**
     * GPS:
     **/

    public static final double CHARGER_XCOORD = 1.78;  // The charger X coordinate.
    public static final double CHARGER_YCOORD = -0.78; // The charger Y coordinate.
    public static final double MAX_GPS_DIST = 5.0;   // The max Euclidean distance from the charger.

    /**
     * V-rep communication:
     **/

    private remoteApi vRep = new remoteApi();
    private int clientID = -1;

    /**
     * Timers:
     **/

    private int MAX_BATT_TIME = 60 * 20; // Default 20 mins battery time.
    private final int MAX_BATT_VOLT = 12;      // volts.

    private Timer motionTimer = new Timer();
    private Timer batteryTimer = new Timer();


    /********************************************************************************************************************
     *                                                   Battery::Methods                                               *
     ********************************************************************************************************************/

    public int getBatteryTime() {
        return (batteryTimer.getSec());
    }

    public void setBatteryTime(int min) {
        MAX_BATT_TIME = 60 * min;
        motionTimer.setSec(MAX_BATT_TIME);
        motionTimer.restart();
    }

    public double getBatteryCapacity() {
        double v = (double) MAX_BATT_VOLT - Utils.map(batteryTimer.getSec(), 0, (double) MAX_BATT_TIME, 0, (double) MAX_BATT_VOLT);
        return ((v > 0.0) ? v : 0.0);
    }

    public double getBatteryPercentage() {
        double v = getBatteryCapacity();

        if ((v >= 9.6) && (v <= 12)) return (100.0);
        else if ((v >= 7.2) && (v < 9.6)) return (80.0);
        else if ((v >= 4.8) && (v < 7.2)) return (60.0);
        else if ((v >= 2.4) && (v < 4.8)) return (40.0);
        else if ((v > 1.0) && (v < 2.4)) return (20.0);
        else
            return (0.0);
    }

    public boolean getBatteryState() {
        double v = getBatteryCapacity();
        return ((v > 0.0) ? true : false);
    }


    /********************************************************************************************************************
     *                                                   Wheel::Methods                                                 *
     ********************************************************************************************************************/

    private double readRightWheelEnc() {
        if (firstRightWheelCall) {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_streaming);
            currentRightJointPosition = robotRightJointPosition.getValue();
            previousRightJointPosition = robotRightJointPosition.getValue();
            firstRightWheelCall = false;
            totalRightJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, rightWheelHandle.getValue(), robotRightJointPosition, remoteApi.simx_opmode_buffer);
            currentRightJointPosition = robotRightJointPosition.getValue();
            dxRight = getAngleMinusAlpha(currentRightJointPosition, previousRightJointPosition);
            totalRightJointPosition += dxRight;
        }
        previousRightJointPosition = currentRightJointPosition;
        return (Math.round((totalRightJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private double readLeftWheelEnc() {
        if (firstLeftWheelCall) {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_streaming);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            previousLeftJointPosition = robotLeftJointPosition.getValue();
            firstLeftWheelCall = false;
            totalLeftJointPosition = 0;
        } else {
            vRep.simxGetJointPosition(clientID, leftWheelHandle.getValue(), robotLeftJointPosition, remoteApi.simx_opmode_buffer);
            currentLeftJointPosition = robotLeftJointPosition.getValue();
            dxLeft = getAngleMinusAlpha(currentLeftJointPosition, previousLeftJointPosition);
            totalLeftJointPosition += dxLeft;
        }
        previousLeftJointPosition = currentLeftJointPosition;

        return (Math.round((totalLeftJointPosition / (2 * Math.PI)) * 100d) / 100d);
    }

    private float getAngleMinusAlpha(float angle, float alpha) {
        double sinAngle0 = Math.sin(angle);
        double sinAngle1 = Math.sin(alpha);
        double cosAngle0 = Math.cos(angle);
        double cosAngle1 = Math.cos(alpha);
        double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
        double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
        return ((float) Math.atan2(sin_da, cos_da));
    }

    public double getLeftWheelEnc() {
        return (encoderValues[0]);
    }

    public double getRightWheelEnc() {
        return (encoderValues[1]);
    }

    public int getEncoderNo() {
        return (2);
    }


    /********************************************************************************************************************
     *                                                   GPS::Methods                                                   *
     ********************************************************************************************************************/

    public double[] readGPS() {
        IntW baseHandle = new IntW(1);
        FloatWA position = new FloatWA(3);
        vRep.simxGetObjectHandle(clientID, "Roomba", baseHandle, remoteApi.simx_opmode_streaming);
        vRep.simxGetObjectPosition(clientID, baseHandle.getValue(), -1, position, remoteApi.simx_opmode_streaming);
        double positions[] = new double[position.getArray().length];

        positions[0] = Math.round((double) position.getArray()[0] * 100.0) / 100.0;
        positions[1] = Math.round((double) position.getArray()[1] * 100.0) / 100.0;
        positions[2] = Math.round((double) position.getArray()[2] * 100.0) / 100.0;

        return (positions);
    }

    public double getGPSX() {
        return (gpsValues[0]);
    }

    public double getGPSY() {
        return (gpsValues[1]);
    }

    public double getGPSZ() {
        return (gpsValues[2]);
    }

    public int getGPSNo() {
        return (3);
    }


    /********************************************************************************************************************
     *                                                   Ultrasonic::Methods                                            *
     ********************************************************************************************************************/

    private double readSonarRange(int sensor) {
        String sensorText = "";

        BoolW detectionState = new BoolW(false);
        FloatWA detectedPoint = new FloatWA(1); //Coordinates relatives to the sensor's frame
        IntW detectedObjectHandle = new IntW(1);
        FloatWA detectedSurfaceNormalVector = new FloatWA(1);

        IntW sensorHandle = new IntW(1);
        switch (sensor) {
            case 0:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor0", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor0Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor0Read = false;
                }
                break;
            case 1:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor1", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor1Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor1Read = false;
                }
                break;
            case 2:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor2", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor2Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor2Read = false;
                }
                break;
            case 3:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor3", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor3Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor3Read = false;
                }
                break;
            case 4:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor4", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor4Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor4Read = false;
                }
                break;
            case 5:
                vRep.simxGetObjectHandle(clientID, "Proximity_sensor5", sensorHandle, remoteApi.simx_opmode_blocking);
                if (!firstSensor5Read) {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_buffer);
                } else {
                    vRep.simxReadProximitySensor(clientID, sensorHandle.getValue(), detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector, remoteApi.simx_opmode_streaming);
                    firstSensor5Read = false;
                }
                break;
        }

        float detectedPointXYZ[] = detectedPoint.getArray();
        double distance = Math.sqrt(Math.pow(detectedPointXYZ[0], 2) + Math.pow(detectedPointXYZ[1], 2) + Math.pow(detectedPointXYZ[2], 2));
        distance = Math.round(distance * 100d) / 100d;
        distance = Utils.getDecimal(distance, "0.0");
        distance = (distance >= 1.0) ? 1.0 : distance;
        distance = (distance == 0.0) ? 1.0 : distance;

        if (detectionState.getValue())
            return (distance);
        return (1.0);
    }

    private double[] readSonars() {
        for (int i = 0; i < getSonarNo(); i++)
            sonarValues[i] = readSonarRange(i);
        return (sonarValues);
    }

    private double[] getSonarRanges() {
        return (sonarValues);
    }

    public double getSonarRange(int sensor) {
        return (sonarValues[sensor]);
    }

    public int getSonarNo() {
        return (sonarValues.length);
    }


    /********************************************************************************************************************
     *                                                   Camera::Methods                                                *
     ********************************************************************************************************************/

    private Color[][] readCamera() {
        if (firstCameraRead) {
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_streaming);
            firstCameraRead = false;
        } else
            vRep.simxGetVisionSensorImage(clientID, cameraHandle.getValue(), resolution, image, 2, remoteApi.simx_opmode_buffer);
        return (imageToColor(image));
    }

    private Color[][] imageToColor(CharWA image) {
        imageArray = image.getArray();
        int index = 0;
        int r, g, b;
        Color color;

        for (int i = 0; i < resolutionCamera; i++)
            for (int j = 0; j < resolutionCamera; j++) {
                // Retrieve the RGB Values:
                r = (int) imageArray[index];
                g = (int) imageArray[index + 1];
                b = (int) imageArray[index + 2];
                color = new Color(r, g, b);
                colorMatrix[i][j] = color;

                bufferedImage.setRGB(i, j, new Color(r, g, b).getRGB());
                index += 3;
            }
        return (colorMatrix);
    }

    private int getGrayscale(BufferedImage img, int x, int y) {
        Color c = new Color(img.getRGB(x, y));
        int r = (int) (c.getRed() * 0.299);
        int g = (int) (c.getGreen() * 0.587);
        int b = (int) (c.getBlue() * 0.114);
        return ((r + g + b));
    }

    private static BufferedImage rotate(BufferedImage bimg, double angle, boolean color) {
        // [1]Get image dimensions:
        int imageType = -1;
        int w = bimg.getWidth();
        int h = bimg.getHeight();

        // [2]Select image type: color/grayscale
        if (color) imageType = bimg.getType();
        else imageType = BufferedImage.TYPE_BYTE_GRAY;

        // [3]Rotate and draw:
        BufferedImage rotated = new BufferedImage(w, h, imageType);
        Graphics2D graphic = rotated.createGraphics();
        graphic.rotate(Math.toRadians(angle), w / 2, h / 2);
        graphic.drawImage(bimg, null, 0, 0);
        graphic.dispose();

        return (rotated);
    }

    public BufferedImage getImage() {
        return (rotate(bufferedImage, -90, false));
    }

    public int getImageWidth() {
        return (bufferedImage.getWidth());
    }

    public int getImageHeight() {
        return (bufferedImage.getHeight());
    }

    public int getImagePixel(int x, int y) {
        return (getGrayscale(bufferedImage, x, y));
    }

    public void setImagePixel(int x, int y, int rgb) {
        bufferedImage.setRGB(x, y, rgb + (rgb << 8) + (rgb << 16));
    }

    public int getTargetX() {
        return (point.x());
    }

    public int getTargetY() {
        return (point.y());
    }

    public double getTargetMinScore() {
        return (targetMinScore);
    }

    public double getTargetMaxScore() {
        return (targetMaxScore);
    }

    public void displayImage() {
        ImageViewer.display(getImage());
    }

    public void templateMatchingCV(BufferedImage image) {
        // [1]Load source and template image files:
        IplImage src = IplImage.create(image.getWidth(), image.getHeight(), opencv_core.IPL_DEPTH_8U, 1);
        src.copyFrom(image);
        IplImage tmp = cvLoadImage("data/images/marker.jpg", 0);

        // [2]The Correlation Image Result:
        IplImage result = cvCreateImage(cvSize(src.width() - tmp.width() + 1, src.height() - tmp.height() + 1), IPL_DEPTH_32F, 1);

        // [3]Select a function template-match method:
        //cvMatchTemplate(src, tmp, result, CV_TM_CCORR_NORMED);  //1*
        cvMatchTemplate(src, tmp, result, CV_TM_CCOEFF_NORMED);   //5*

        double min_val[] = new double[2];
        double max_val[] = new double[2];

        // [4]Max and Min correlation point locations:
        CvPoint minLoc = new CvPoint();
        CvPoint maxLoc = new CvPoint();

        // [5]Compute and print min-max value locations:
        cvMinMaxLoc(result, min_val, max_val, minLoc, maxLoc, null);
        targetMinScore = min_val[0]; // Min Score.
        targetMaxScore = max_val[0]; // Max Score.
        //System.out.println("Min: " + targetMin);
        //System.out.println("Max: " + targetMax);

        // [6]Mark at point the image template coordinates:
        point.x(maxLoc.x() + tmp.width());
        point.y(maxLoc.y() + tmp.height());

        // [7]Draw the rectangle result in source image:
        cvRectangle(src, maxLoc, point, CvScalar.GRAY, 2, 8, 0);

        // [8]Display the image:
        ImageViewer.display(src.getBufferedImage());
    }


    /********************************************************************************************************************
     *                                                   Motion::Methods                                                *
     ********************************************************************************************************************/

    public void forward() {
        resetButtonsStyle();
        btnForward.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'f';
    }

    public void backward() {
        resetButtonsStyle();
        btnBack.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'b';
    }

    public void left() {
        resetButtonsStyle();
        btnLeft.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'l';
    }

    public void right() {
        resetButtonsStyle();
        btnRight.setStyle("-fx-background-color: #7FFF00; ");
        dir = 'r';
    }

    public void stop() {
        resetButtonsStyle();
        btnStop.setStyle("-fx-background-color: #7FFF00; ");
        dir = 's';
    }

    private void resetButtonsStyle() {
        btnRight.setStyle(defaultButtonStyle);
        btnStop.setStyle(defaultButtonStyle);
        btnLeft.setStyle(defaultButtonStyle);
        btnForward.setStyle(defaultButtonStyle);
        btnBack.setStyle(defaultButtonStyle);
    }

    public void setVel(float lVel, float rVel) {
        vRep.simxSetJointTargetVelocity(clientID, leftWheelHandle.getValue(), lVel, remoteApi.simx_opmode_oneshot);
        vRep.simxSetJointTargetVelocity(clientID, rightWheelHandle.getValue(), rVel, remoteApi.simx_opmode_oneshot);
    }

    public void move(float vel) {
        setVel(vel, vel);
    }

    public void move(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            move(vel);
        }
        stop();
    }

    public void turnSpot(float vel) {
        setVel(vel, -vel);
    }

    public void turnSpot(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSpot(vel);
        }
        stop();
    }

    public void turnSharp(float vel) {
        if (vel > 0) setVel(vel, 0);
        else setVel(0, -vel);
    }

    public void turnSharp(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSharp(vel);
        }
        stop();
    }

    public void turnSmooth(float vel) {
        if (vel > 0) setVel(vel, vel / 2);
        else setVel(-vel / 2, -vel);
    }

    public void turnSmooth(float vel, int time) {
        motionTimer.setMs(time);
        motionTimer.restart();
        while (motionTimer.getState()) {
            turnSmooth(vel);
        }
        stop();
    }

    public void teleoperate(char dir, int vel) {
        switch (dir) {
            case 's':
                move(vel = 0);
                break;
            case 'f':
                move(+vel);
                break;
            case 'b':
                move(-vel);
                break;
            case 'r':
                turnSpot(+vel / 2);
                break;
            case 'l':
                turnSpot(-vel / 2);
                break;
        }
    }


    /********************************************************************************************************************
     *                                                   Generic::Methods                                               *
     ********************************************************************************************************************/

    public void connectToVrep() {
        clientID = vRep.simxStart("127.0.0.1", 20001, true, true, 5000, 5);
        if (clientID == -1) {
            btnConnect.setText("Failed");
            btnConnect.setStyle("-fx-background-color: #FF0000; ");
            vRep.simxFinish(clientID);
        } else {
            btnConnect.setStyle("-fx-background-color: #7FFF00; ");
            btnConnect.setText("Connected");
            setup();
        }
    }

    public void setup() {
        vRep.simxGetObjectHandle(clientID, "JointLeftWheel", leftWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "JointRightWheel", rightWheelHandle, remoteApi.simx_opmode_blocking);
        vRep.simxGetObjectHandle(clientID, "Vision_sensor", cameraHandle, remoteApi.simx_opmode_blocking);

        defaultButtonStyle = btnForward.getStyle();
        pw = canvasCamera.getGraphicsContext2D().getPixelWriter();
        ImageViewer.open(resolutionCamera, resolutionCamera, "Camera");

        motionTimer.setSec(1);
        batteryTimer.setSec(MAX_BATT_TIME);
        motionTimer.start();
        batteryTimer.start();

        update.start();
        main.start();
    }

    /**
     * Method     : Controller::update()
     * Purpose    : To update the robot.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    private Thread update = new Thread() {
        public void run() {
            setBatteryTime(20);
            while (true) {
                // [1]Update robot:
                if (runGPS) {
                    gpsValues = readGPS();
                }
                if (runSensors) {
                    sonarValues = readSonars();
                }
                if (runWheelEncoder) {
                    encoderValues[0] = readLeftWheelEnc();
                    encoderValues[1] = readRightWheelEnc();
                }
                if (runCamera) {
                    imageCamera = readCamera();
                }
                if (runMotion) {
                    teleoperate(dir, vel);
                }
                Platform.runLater(new Runnable() {
                    public void run() {
                        if (runGPS) {
                            lblGpsX.setText("X: " + gpsValues[0]);
                            lblGpsY.setText("Y: " + gpsValues[1]);
                            lblGpsZ.setText("Z: " + gpsValues[2]);
                        }
                        if (runSensors) {
                            lblSensor0.setText(sonarValues[0] + "m");
                            lblSensor1.setText(sonarValues[1] + "m");
                            lblSensor2.setText(sonarValues[2] + "m");
                            lblSensor3.setText(sonarValues[3] + "m");
                            lblSensor4.setText(sonarValues[4] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                            lblSensor5.setText(sonarValues[5] + "m");
                        }
                        if (runWheelEncoder) {
                            lblRightWheel.setText("Right : " + encoderValues[0]);
                            lblLeftWheel.setText("Left : " + encoderValues[1]);
                        }
                    }
                });

                // [2]Update custom code:
                update();

                // [3]Update battery:
                if (!getBatteryState()) {
                    System.err.println("Error: Robot out of battery...");
                    move(0, 1000);
                    running = false;
                    break;
                }
                Delay.ms(1);
            }
        }
    };

    /**
     * Method     : Controller::main()
     * Purpose    : To run the main code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    private Thread main = new Thread() {
        public void run() {


            while (true) {
                main();
                Delay.ms(1);
            }
        }
    };

    /********************************************************************************************************************
     *                                                   Student Code                                                   *
     ********************************************************************************************************************/

//    private FSM avoid = new Avoid(3, 100);
//    private FSM track  = new Track( 75, 3);
//    private FSM clean  = new Clean( 50, 3);
//    private FSM wander = new Wander(3, 25);
    // --- Détection de blocage (stuck) ---
    private Double stuckRefDist = null;  // distance de référence devant
    private long   stuckSinceMs = 0;     // depuis quand on voit ~la même distance

    // --- Détection de blocage "pas de progression" ---
    private double lastProgX = 0.0, lastProgY = 0.0;
    private long   lastProgTs = 0;
    private boolean progInit = false;


    /**
     * Method     : Controller::update()
     * Purpose    : To update custom code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : Comment where appropriate.
     **/
    public void update() {
        // Read from image test:
//        int x = getImageWidth()/2;
//        int y = getImageHeight()/2;
//        System.out.println("pixel[" + x + "," + y + "]: " + getImagePixel(x, y));
//        System.out.println("target[" + getTargetX() + "," + getTargetY() + "]: " + getTargetMaxScore());

        // Write on image test:
//        for(x=0 ; x<(getImageWidth()/3)  ; x++)
//        for(y=0 ; y<(getImageHeight()/3) ; y++)
//        setImagePixel(x, y, 128);
//        displayImage();

        // Template matching:
        templateMatchingCV(getImage());

        // Print sensors test:
//        System.out.println("\nWheel(R): " + getRightWheelEnc() + ", Wheel(L): " + getLeftWheelEnc()); // Wheel revolutions.
//        System.out.println("GPS(X): " + getGPSX() + ", GPS(Y): " + getGPSY() + ", GPS(Z): " + getGPSZ()); // GPS coordinates.
//        System.out.println("C: " + getBatteryCapacity() + "v, P: " + getBatteryPercentage() + "%, S: " + getBatteryState() + ", T: " + getBatteryTime() + "sec"); // Battery stats.
//        for(int i=0 ; i<getSonarNo() ; i++) System.out.println(i + ": " + Utils.getDecimal(getSonarRange(i), "0.0")); // Print ultrasonic ranges.
    }

    /**
     * Method     : Controller::main()
     * Purpose    : To run the main code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/
    public void main()
    {
        run();
    }

    /**
     * Method     : SubsumptionCoordinator::run()
     * Purpose    : To run a custom Subsumption Architecture.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : None.
     **/

    private double dist2D(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1, dy = y2 - y1;
        return Math.sqrt(dx*dx + dy*dy);
    }


    public void run() {
        Integer priority[] = new Integer[2];
        //        int priority[] = new int[4];

        double cam = getTargetMaxScore();                                                      // Target horizontal detection (pixels).
        double bat = getBatteryCapacity();                                                     // Battery capacity (volts).
        double snr = Arrays.stream(getSonarRanges()).min().getAsDouble();                      // Min sonar range radius (meters).
        double gps = Utils.getEuclidean(CHARGER_XCOORD, getGPSY(), getGPSX(), CHARGER_YCOORD); // GPS distance from charger (meters?).
        double sensors[] = new double[]{bat, snr, cam, gps};                                   // Sensor vector.

        double sonarData[] = new double[]
                {
                        getSonarRange(0),
                        getSonarRange(1),
                        getSonarRange(2),
                        getSonarRange(3),
                        getSonarRange(4),
                        getSonarRange(5)
                };

        if (dir == 's') { // AUTO uniquement quand Stop est actif

            // --- Réglages ---
            final double ENTER_TH     = 0.30;  // axe avant "bouché" si < 0.30 m
            final double PASS_MARGIN  = 0.25;  // couloir acceptable sur au moins un côté
            final float  CRUISE       = 2.0f;  // avance normale
            final float  CRUISE_SLOW  = 1.2f;  // avance prudente

            // Détection de STALL (pas de progression)
            final double PROG_EPS     = 0.02;  // 2 cm minimum de déplacement
            final long   PROG_TIMEOUT = 2000;  // 2 s sans progresser => bloqué
            final int    ESC_BACK_MS  = 1000;   // recul ms
            final float  ESC_BACK_VEL = -1.2f; // vitesse de recul
            final int    ESC_TURN_MS  = 750;   // ~90° (ajuste si besoin)
            final float  ESC_TURN_VEL =  vel/3;// un peu plus vif que vel/4 pour se décoincer

            // Mapping capteurs
            final int FRONT_SENSOR = 2; // si chez toi c'est 3, mets 3


            // 1) Distances utiles
            double frontAxis = getSonarRange(FRONT_SENSOR);
            double minLeft   = Math.min(Math.min(getSonarRange(0), getSonarRange(1)), getSonarRange(2));
            double minRight  = Math.min(Math.min(getSonarRange(3), getSonarRange(4)), getSonarRange(5));

            // 2) Détection "pas de progression" (via GPS ; tu peux utiliser l’odométrie si tu préfères)
            double x = getGPSX(), y = getGPSY();
            long now = System.currentTimeMillis();

            if (!progInit) { // init au premier passage
                lastProgX = x; lastProgY = y; lastProgTs = now; progInit = true;
            }

            boolean noProgress = false;
            double d = dist2D(lastProgX, lastProgY, x, y);
            if (d >= PROG_EPS) {
                // on a avancé : reset du timer
                lastProgX = x; lastProgY = y; lastProgTs = now;
            } else {
                // pas (ou peu) de déplacement : checker le timeout
                if (now - lastProgTs >= PROG_TIMEOUT) {
                    noProgress = true;
                    // on reset pour éviter déclenchements en chaîne
                    lastProgX = x; lastProgY = y; lastProgTs = now;
                }
            }

            // 3) Si pas de progression -> manœuvre d’échappement
            if (noProgress) {
                // --- paramètres de la séquence d’échappement ---
                final int   PAUSE_MS        = 150;     // pause après le recul / après le pivot
                final float CRUISE_RESUME   = 1.6f;    // poussée d’élan après le pivot
                final int   RESUME_MS       = 600;     // durée de la poussée
                // (ESC_BACK_MS, ESC_BACK_VEL, ESC_TURN_MS, ESC_TURN_VEL déjà définis au-dessus)

                // 1) reculer pour se dégager
                move(ESC_BACK_VEL, ESC_BACK_MS);

                // 2) pause courte (laisser le robot se stabiliser)
                move(0f, PAUSE_MS);

                // 3) tourner ~90° vers le côté le plus dégagé
                //    => on choisit le côté dont la distance minimale est la PLUS GRANDE
                boolean turnRight = (minRight >= minLeft);
                if (turnRight) {
                    // droite plus dégagée -> tourne à droite
                    turnSpot(+ESC_TURN_VEL, ESC_TURN_MS);
                } else {
                    // gauche plus dégagée -> tourne à gauche
                    turnSpot(-ESC_TURN_VEL, ESC_TURN_MS);
                }

                // 4) pause courte après le pivot
                move(0f, PAUSE_MS);

                // 5) relance vers l’avant (petit "push" pour sortir de la zone)
                move(CRUISE_RESUME, RESUME_MS);

                // 6) réinitialiser la référence de progression pour éviter un retrigger immédiat
                lastProgX = getGPSX();
                lastProgY = getGPSY();
                lastProgTs = System.currentTimeMillis();

                return; // fin du cycle
            }

            // 4) Logique normale (cône + couloir)
            if (frontAxis >= ENTER_TH) {
                // Axe plein-avant libre -> on avance, même si ça frotte sur les côtés
                move(CRUISE);
            } else {
                // Avant proche : s'il existe un couloir d'au moins un côté, on passe doucement
                if (minLeft > PASS_MARGIN || minRight > PASS_MARGIN) {
                    move(CRUISE_SLOW);
                } else {
                    // Vraie obstruction -> éviter
                    avoid();
                }
            }
        }
    }
    public void avoid()
    {
        double leftMinSonarRadius  = Arrays.stream(new double[]{getSonarRange(0), getSonarRange(1), getSonarRange(2)}).min().getAsDouble();
        double rightMinSonarRadius = Arrays.stream(new double[]{getSonarRange(3), getSonarRange(4), getSonarRange(5)}).min().getAsDouble();

        if(leftMinSonarRadius < rightMinSonarRadius)
        {
            turnSpot(vel/4, 500);
        }
        else
        if(leftMinSonarRadius > rightMinSonarRadius)
        {
            turnSpot(-vel/4, 500);
        }
    }

    /**
     * Method     : Controller::tlu()
     * Purpose    : To implement a Threshold Logic Unit.
     * Parameters : - w_vec : The weight vector.
     * - w_vec : The sensor vector.
     * - f     : The activation threshold.
     * Returns    : True (+1) if TLU is activated, False (-1) otherwise.
     * Notes      : None.
     **/
    public boolean tlu(double w_vec[], double s_vec[], double f)
    {
        return (false);
    }
}
