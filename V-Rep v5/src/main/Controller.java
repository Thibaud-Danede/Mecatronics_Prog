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
        updateModeUI();
        updateTrackOverrideUI();
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

    // --- Waypoints pour la phase 1 du Track (GPS uniquement) ---
    // À REMPLACER par tes vraies coordonnées (en mètres, dans le repère du GPS Coppelia)
    private static final double[][] TRACK_WAYPOINTS = {
            { CHARGER_XCOORD, CHARGER_YCOORD }  // un seul waypoint : le dock
    };


    private static final double WP_REACHED_DIST = 0.25; // rayon pour considérer un WP atteint

    private int    trackCurrentWp        = 0;
    private double trackLastDistToTarget = Double.MAX_VALUE;


    // --- État Track phase 1 (retour GPS) ---
    private double  trackLastDistToDock  = Double.MAX_VALUE;


    // Compteur pour détecter quand Avoid est lui-même coincé
    private int avoidStuckCounter = 0;


    // Indique si on est actuellement en phase 2 du track (approche dock)
    private boolean inTrackPhase2 = false;


    // Indique si on considère que le robot est bien docké
    private boolean trackDocked = false;


    // --- Odométrie simple avec les encodeurs ---
    private boolean odoInit = false;
    private double lastLeftEnc  = 0.0;
    private double lastRightEnc = 0.0;
    private double odoTheta     = 0.0; // orientation estimée (rad)

    // Paramètres mécaniques approximatifs du Roomba
    private static final double WHEEL_RADIUS = 0.035; // ~3.5 cm
    private static final double WHEEL_BASE   = 0.27;  // ~27 cm entre les roues


    // --- Position de la station de charge (dock) ---
    private boolean dockPositionInitialized = true;
    private double dockX = CHARGER_XCOORD;  // valeur de secours
    private double dockY = CHARGER_YCOORD;  // valeur de secours

    // Seuils pour le comportement Track
    // Au-delà de TRACK_NEAR_DIST : on est "loin" → phase GPS
    // En dessous de TRACK_NEAR_DIST : on est dans la zone de docking → phase caméra
    // En dessous de TRACK_DOCKED_DIST : on considère qu'on est docké
    private static final double TRACK_NEAR_DIST   = 0.35;   // m
    private static final double TRACK_DOCKED_DIST = 0.20;  // m
    // Par exemple, 8 cm de marge en plus
    private static final double TRACK_DOCKED_MARGIN = 0.08;

    // Pour vérifier que la pose de docking reste bonne sur une courte durée
    private long dockStableSinceMs = 0;

    // Détection de blocage spécifique à la phase 2 du Track (caméra)
    private long trackP2StuckSinceMs = 0;
    private long trackP2LastSpinMs   = 0;

    // Pour savoir si on a déjà fait le scan initial en phase 2
    private boolean trackP2InitialScanDone = false;





    // Pour n'appliquer l'override de batterie qu'une seule fois
    private boolean batteryDevOverrideDone = false;

    // Log batterie périodique
    private long lastBatteryLogTimeMs = 0;

    // Phase de démarrage (pour se dégager du mur initial)
    private boolean startupDone = false;


    // --- FSM pour Clean (pattern rectangulaire) ---
    private static final int CLEAN_STATE_INIT    = 0;
    private static final int CLEAN_STATE_FORWARD = 1;
    private static final int CLEAN_STATE_TURN    = 2;

    private int    cleanState      = CLEAN_STATE_INIT;
    private int    cleanSideIndex  = 0;       // 0..3 pour 4 côtés du rectangle
    private double cleanStartX     = 0.0;
    private double cleanStartY     = 0.0;

    // Paramètres du motif rectangulaire (à ajuster en simulation)
    private static final double CLEAN_SIDE_LENGTH   = 1.0;  // longueur d’un côté (en mètres environ)
    private static final int    CLEAN_STEP_TIME_MS  = 150;  // durée d’un “pas” en ligne droite
    private static final int    CLEAN_TURN_TIME_MS  = 1600;  // temps approximatif pour tourner 90°

    // --- Stubs pour éviter les erreurs si le FXML appelle ces méthodes
    //     (ne dépendent d’aucune variable externe ; sans effet si le bouton n’existe pas) ---
    //
    // Détection de blocage (stuck)
    private double lastStuckCheckX = 0.0;
    private double lastStuckCheckY = 0.0;
    private long   lastStuckCheckTimeMs = 0;
    private int    stuckCounter = 0;



    // --- MODES (utiles pour le cycle MANU → ROUTE → AUTO) ---
    private static final int MODE_AUTO  = 0;
    private static final int MODE_MANU  = 1;

    // --- IDs de comportements (pour le debug) ---
    private static final int BEH_NONE   = 0;
    private static final int BEH_AVOID  = 1;
    private static final int BEH_TRACK  = 2;
    private static final int BEH_CLEAN  = 3;
    private static final int BEH_WANDER = 4;

    // Comportement actuellement actif (pour détecter les changements)
    private int currentBehavior = BEH_NONE;

    // État courant du mode. Tu peux choisir le démarrage que tu veux.
    // Pour coller au label initial du FXML ("Mode: AUTO"), mets MODE_AUTO.
    // Si tu préfères démarrer MANU, mets MODE_MANU.
    private int mode = MODE_AUTO;

    @FXML private Button btnMode; // si déjà déclaré ailleurs, supprime cette ligne

    @FXML private Button btnForceTrack;
    // Forçage manuel du comportement TRACK (via un bouton GUI)
    private boolean trackOverride = false;


    // Met à jour le libellé du bouton selon le mode courant
    private void updateModeUI() {
        if (btnMode == null) return;
        String label = (mode == MODE_AUTO) ? "Mode: AUTO" : "Mode: MANU";
        btnMode.setText(label);
    }

    // Met à jour le libellé du bouton TRACK forcé
    private void updateTrackOverrideUI() {
        if (btnForceTrack == null) return;
        String label = trackOverride ? "TRACK forcé: ON" : "TRACK forcé: OFF";
        btnForceTrack.setText(label);
    }


    @FXML
    private void toggleMode() {
        if (mode == MODE_AUTO) {
            mode = MODE_MANU;
            System.out.println("Mode -> MANU (pilotage manuel)");
        } else {
            mode = MODE_AUTO;
            System.out.println("Mode -> AUTO (subsomption)");
        }
        updateModeUI();
    }

    @FXML
    private void toggleTrackOverride() {
        trackOverride = !trackOverride;

        if (trackOverride) {
            System.out.println("[DEBUG] TRACK forcé activé (le TLU Track est ignoré).");
        } else {
            System.out.println("[DEBUG] TRACK forcé désactivé (retour au TLU normal).");
        }

        updateTrackOverrideUI();
    }


    // Retourne true quand la phase de démarrage est terminée
    private boolean startupStep() {
        // On regarde "devant" : les sonars 1 et 4 (légèrement orientés)
        double frontLeft  = getSonarRange(1);
        double frontRight = getSonarRange(4);
        double frontMin   = Math.min(frontLeft, frontRight);

        final double SAFE_DIST = 0.6; // distance "confortable" à dégager (à ajuster)

        if (frontMin < SAFE_DIST) {
            // Trop près du mur : on tourne sur place pour élargir l'angle
            System.out.println("[STARTUP] Trop près du mur, rotation d'évitement");
            // Ici on tourne sur place : pas de risque de rentrer dans le mur
            turnSpot(vel/2.0f, 3200);
            return false; // pas encore fini
        } else {
            // On considère qu'on est suffisamment dégagé pour lancer le mode AUTO normal
            System.out.println("[STARTUP] Démarrage terminé, passage en AUTO classique");
            return true;
        }
    }



    // Comportement Clean : motif rectangulaire simple
    public void clean() {
        switch (cleanState) {
            case CLEAN_STATE_INIT:
                // On mémorise la position de départ de ce côté
                cleanStartX = getGPSX();
                cleanStartY = getGPSY();
                cleanState  = CLEAN_STATE_FORWARD;
                break;

            case CLEAN_STATE_FORWARD:
                // Distance parcourue depuis le début de ce côté
                double dx   = getGPSX() - cleanStartX;
                double dy   = getGPSY() - cleanStartY;
                double dist = Math.sqrt(dx * dx + dy * dy);

                if (dist < CLEAN_SIDE_LENGTH) {
                    // On continue tout droit par petits pas
                    move(vel * 0.8f, CLEAN_STEP_TIME_MS);
                } else {
                    // On a atteint la longueur voulue, on passera au virage
                    cleanState = CLEAN_STATE_TURN;
                }
                break;

            case CLEAN_STATE_TURN:
                // Tour sur place d’environ 90°
                // (CLEAN_TURN_TIME_MS est à ajuster pour un quart de tour)
                turnSpot(vel / 2.0f, CLEAN_TURN_TIME_MS);

                // On passe au côté suivant
                cleanSideIndex = (cleanSideIndex + 1) % 4;

                // Et on recommence un nouveau segment
                cleanState = CLEAN_STATE_INIT;
                break;

            default:
                // Sécurité : on réinitialise si jamais on tombe dans un état inconnu
                cleanState = CLEAN_STATE_INIT;
                break;
        }
    }


    public void wander() {
        System.out.println("[AUTO] Wander: manœuvre aléatoire");

        // Choix aléatoire gauche/droite
        double rnd = Math.random();
        float turnVel = vel / 2.0f;
        int turnTime = 300 + (int)(Math.random() * 700); // 300-1000 ms

        if (rnd < 0.5) {
            // tourne à gauche
            turnSpot(-turnVel, turnTime);
        } else {
            // tourne à droite
            turnSpot(turnVel, turnTime);
        }

        // Petite avance après la rotation
        move(vel * 0.8f, 400);
    }


    /**
     * Method     : Controller::update()
     * Purpose    : To update custom code.
     * Parameters : None.
     * Returns    : Nothing.
     * Notes      : Comment where appropriate.
     **/
    public void update() {
        // Read from image test:
//    int x = getImageWidth()/2;
//    int y = getImageHeight()/2;
//    System.out.println("pixel[" + x + "," + y + "]: " + getImagePixel(x, y));
//    System.out.println("target[" + getTargetX() + "," + getTargetY() + "]: " + getTargetMaxScore());

        // Write on image test:
//    for(x=0 ; x<(getImageWidth()/3)  ; x++)
//    for(y=0 ; y<(getImageHeight()/3) ; y++)
//    setImagePixel(x, y, 128);
//    displayImage();

        // Template matching:
        templateMatchingCV(getImage());

        // Print sensors test:
//    System.out.println("\nWheel(R): " + getRightWheelEnc() + ", Wheel(L): " + getLeftWheelEnc()); // Wheel revolutions.
//    System.out.println("GPS(X): " + getGPSX() + ", GPS(Y): " + getGPSY() + ", GPS(Z): " + getGPSZ()); // GPS coordinates.
//    System.out.println("C: " + getBatteryCapacity() + "v, P: " + getBatteryPercentage() + "%, S: " + getBatteryState() + ", T: " + getBatteryTime() + "sec"); // Battery stats.
//    for(int i=0 ; i<getSonarNo() ; i++) System.out.println(i + ": " + Utils.getDecimal(getSonarRange(i), "0.0")); // Print ultrasonic ranges.
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
    public void run()
    {
        // 0) Si on est déjà docké, on coupe tout comportement automatique
        if (trackDocked) {
            // On s'assure que les moteurs sont bien à l'arrêt
            setVel(0, 0);
            dir = 's';

            // Optionnel : on log une seule fois le passage en état "dock"
            if (currentBehavior != BEH_NONE) {
                System.out.println("[AUTO] Dock atteint -> arrêt définitif des comportements automatiques.");
                currentBehavior = BEH_NONE;
            }

            // On ne fait plus AUCUNE décision auto tant que trackDocked reste true
            return;
        }

        // Mise à jour de l'orientation estimée à partir des encodeurs
        updateOdometry();

        // 1) Initialiser la position de docking une seule fois au début
        if (!dockPositionInitialized) {
            // On connaît déjà la position fixe du dock dans la scène
            dockX = CHARGER_XCOORD;   // 1.78
            dockY = CHARGER_YCOORD;   // -0.78
            dockPositionInitialized = true;

            System.out.println("[DOCK] Position fixe de la station: X=" + dockX + "  Y=" + dockY);
        }


        // Override dev : batterie plus courte (ex: 2 minutes)
//        if (!batteryDevOverrideDone) {
//            setBatteryTime(2);  // 2 minutes au lieu de 20
//            batteryDevOverrideDone = true;
//            System.out.println("[BATT] Override dev: batterie réglée sur 2 minutes.");
//        }

        Integer priority[] = new Integer[2];
        //        int priority[] = new int[4];

        double cam = getTargetMaxScore();                                                      // Target horizontal detection (pixels).
        double bat = getBatteryCapacity();                                                     // Battery capacity (volts).
        double snr = Arrays.stream(getSonarRanges()).min().getAsDouble();                      // Min sonar range radius (meters).

        // On utilise la position dynamique du dock si elle est connue,
        // sinon on retombe sur les constantes d'origine.
        double dockXUsed = dockPositionInitialized ? dockX : CHARGER_XCOORD;
        double dockYUsed = dockPositionInitialized ? dockY : CHARGER_YCOORD;

        double gps = Utils.getEuclidean(
                getGPSX(), getGPSY(),
                dockXUsed, dockYUsed
        );
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

        // --- Log batterie toutes les ~10 secondes ---
        long now = System.currentTimeMillis();
        if (now - lastBatteryLogTimeMs >= 10_000) { // 10 000 ms = 10 s
            lastBatteryLogTimeMs = now;

            double v  = getBatteryCapacity();
            double pc = getBatteryPercentage();
            int    t  = getBatteryTime(); // temps écoulé en secondes

            System.out.printf(
                    "[BATT] V=%.2f V, P=%.0f%%, t=%d s%n",
                    v, pc, t
            );
        }

        // --- Sélection par mode ---
        switch (mode) {
            case MODE_AUTO:
                // Utilisation des TLUs pour choisir Avoid / Track / etc.
                autoModeWithTLU(cam, bat, snr, gps);
                break;
            case MODE_MANU:
            default:
                // En manuel : on laisse l'utilisateur piloter via l'UI.
                // Si tu veux un filet de sécurité, décommente la ligne suivante :
                // avoid();
                break;
        }
    }


    /**
     * Mode AUTO : sélection de comportement via TLUs + subsomption.
     *
     * capteurs bruts :
     *  - cam : score de template matching ([-1,1] typiquement)
     *  - bat : capacité batterie (0..12 V)
     *  - snr : distance min sonar (0..1 m, 1 = loin ou rien détecté)
     *  - gps : distance au chargeur (en m)
     */
    private void autoModeWithTLU(double cam, double bat, double snr, double gps)
    {
        // ---------- Phase de démarrage : se dégager du mur initial ----------
        if (!startupDone) {
            // Tant que startupStep() renvoie false, on ne fait que ça
            if (startupStep()) {
                startupDone = true;
            }
            return; // on ne lance pas encore les TLUs / subsomption
        }

        // ---------- Normalisation des capteurs dans [0,1] ----------

        // Batterie
        double batNorm = clamp01(Utils.map(bat, 0.0, (double) MAX_BATT_VOLT, 0.0, 1.0));
        double batLow  = 1.0 - batNorm; // 0 = batterie pleine, 1 = batterie très faible

        // GPS : distance au dock -> "near"
        double gpsClamped = Math.min(gps, MAX_GPS_DIST);
        double gpsNear    = 1.0 - (gpsClamped / MAX_GPS_DIST);
        gpsNear = clamp01(gpsNear);

        // Caméra : score [-1,1] -> [0,1]
        double camNorm = clamp01((cam + 1.0) / 2.0);

        // ---------- Sonars pour Avoid : on ne regarde que les frontaux (2 et 3) ----------

        double[] ranges = getSonarRanges();
        double s2 = ranges[2]; // avant-gauche
        double s3 = ranges[3]; // avant-droite

        // distance la plus proche DEVANT
        double snrFront   = Math.min(s2, s3);
        double snrClamped = clamp01(snrFront);
        double snrClose   = 1.0 - snrClamped;   // 0 = loin, 1 = très proche

        // ---------- TLU Avoid ----------
        // On veut que Avoid s'active quand un obstacle est à moins de ~0.45 m devant
        // THRESH_FRONT = 0.45 dans avoid()
        // snrClose = 1 - snrFront > fAvoid  <=>  snrFront < 1 - fAvoid
        // Pour snrFront < 0.45, on prend fAvoid ≈ 1 - 0.45 = 0.55
        double[] sAvoid = { snrClose };
        double[] wAvoid = { 1.0 };
        double   fAvoid = 0.55;

        boolean avoidActive = tlu(wAvoid, sAvoid, fAvoid);

        // ---------- TLU Track : s'active quand la batterie est faible ----------

        // batLow ~ 0.0 => batterie pleine
        // batLow ~ 1.0 => batterie très faible
        double[] sTrack = { batLow };
        double[] wTrack = { 1.0 };

        // Avec fTrack = 0.2 :
        // Track s'active quand batLow > 0.2, c.-à-d. quand la batterie est <~ 80 %
        double fTrack = 0.2;

        boolean trackActive = tlu(wTrack, sTrack, fTrack);

        // Si le bouton "TRACK forcé" est activé, on force trackActive à true
        if (trackOverride) {
            trackActive = true;
        }

        // ---------- Clean & Wander ----------
        // Clean = comportement de fond ; Wander = si bloqué
        boolean isStuckNow = isStuck();

        // ---------- Subsomption : (Track phase 2) > Avoid > Track > Clean > Wander ----------
        int newBehavior;

        if (inTrackPhase2) {
            // En phase 2 de Track, on ne laisse plus Avoid prendre la main.
            newBehavior = BEH_TRACK;
        }
        else if (avoidActive) {
            newBehavior = BEH_AVOID;
        }
        else if (trackActive) {
            newBehavior = BEH_TRACK;
        }
        else if (!isStuckNow) {
            newBehavior = BEH_CLEAN;
        }
        else {
            newBehavior = BEH_WANDER;
        }

        // Log si changement
        logBehaviorChangeIfNeeded(newBehavior);

        // Exécution
        switch (newBehavior) {
            case BEH_AVOID:
                avoid();
                break;

            case BEH_TRACK:
                track();
                break;

            case BEH_WANDER:
                wander();
                break;

            case BEH_CLEAN:
            default:
                clean();
                break;
        }
    }



    public void track()
    {
        if (trackDocked) {
            setVel(0, 0);
            dir = 's';
            inTrackPhase2 = false;
            return;
        }

        if (!dockPositionInitialized) {
            System.out.println("[TRACK] Dock non initialisé, fallback: avance lente.");
            inTrackPhase2 = false;
            move(vel * 0.5f, 200);
            return;
        }

        double x = getGPSX();
        double y = getGPSY();
        System.out.println(String.format(
                "[DEBUG TRACK] GPS=(%.2f, %.2f) dock=(%.2f, %.2f) dist=%.2f",
                x, y, dockX, dockY, Utils.getEuclidean(x, y, dockX, dockY)
        ));


        double distToDock = Utils.getEuclidean(x, y, dockX, dockY);

        boolean wasInPhase2 = inTrackPhase2;

        // PHASE 1 : loin du dock -> navigation par waypoints GPS
        if (distToDock > TRACK_NEAR_DIST) {
            inTrackPhase2 = false;
            trackReturnToDockWithWaypoints(x, y, distToDock);
        }
        // PHASE 2 : proche du dock -> caméra
        else {
            inTrackPhase2 = true;

            if (!wasInPhase2) {
                System.out.println("[TRACK] Passage en phase 2 (caméra).");
                trackLastDistToTarget   = Double.MAX_VALUE;
                trackP2InitialScanDone  = false;
                dockStableSinceMs       = 0;
                trackP2StuckSinceMs     = 0L;
            }

            trackDockingApproach(distToDock);
        }
    }


    /// Phase 1 du Track : retour vers le dock en utilisant UNIQUEMENT le GPS.
    /// On ignore les waypoints : la cible est toujours (dockX, dockY).
    private void trackReturnToDockWithWaypoints(double x, double y, double distToDock)
    {
        // 0) Si le robot est considéré comme bloqué, on laisse Wander essayer de le débloquer.
        if (isStuck()) {
            System.out.println("[TRACK P1] Robot bloqué -> appel Wander.");
            wander();
            return;
        }

        // 1) Cible = position du dock
        double targetX = dockX;
        double targetY = dockY;

        // 2) Vecteur vers le dock
        double dx = targetX - x;
        double dy = targetY - y;
        double distToTarget = Math.sqrt(dx * dx + dy * dy);

        // Sécurité : si on est déjà "près" (normalement track() passera en phase 2)
        if (distToTarget < TRACK_NEAR_DIST) {
            System.out.println(String.format(
                    "[TRACK P1] Déjà proche du dock (%.2f m) -> pas de mouvement (phase 2 va prendre la main).",
                    distToTarget
            ));
            setVel(0, 0);
            return;
        }

        // 3) Angle absolu vers le dock
        double desiredHeading = Math.atan2(dy, dx);  // [-pi, +pi]

        // 4) Erreur d'angle entre où on regarde (odoTheta) et où est le dock
        double headingError = normalizeAngle(desiredHeading - odoTheta);

        // 5) Seuils / vitesses
        final double ANGLE_TOL = Math.toRadians(10.0);  // tolérance de 10°
        float turnVel  = vel / 2.0f;    // vitesse de rotation
        int   turnTime = 200;          // durée d'une impulsion de rotation (ms)
        float fwdVel   = vel * 0.8f;   // vitesse d'avance
        int   moveTime = 200;          // durée d'une impulsion d'avance (ms)

        // 6) Log de debug (très utile au début)
        System.out.println(String.format(
                "[TRACK P1] pos=(%.2f, %.2f) dock=(%.2f, %.2f) dist=%.2f  theta=%.2f rad  err=%.2f rad",
                x, y, targetX, targetY, distToTarget, odoTheta, headingError
        ));

        // 7) Décision : tourner ou avancer
        if (Math.abs(headingError) > ANGLE_TOL) {
            // On doit corriger l'orientation avant d'avancer.

            if (headingError > 0) {
                // Dock à "gauche" -> on veut AUGMENTER l'angle -> tourner sur place dans ce sens
                System.out.println("[TRACK P1] Dock à gauche -> rotation gauche.");
                // NB : turnSpot(vel) fait décroître l'angle (voir updateOdometry),
                // donc pour augmenter l'angle on utilise -turnVel.
                turnSpot(-turnVel, turnTime);
            } else {
                // Dock à "droite" -> on veut DIMINUER l'angle
                System.out.println("[TRACK P1] Dock à droite -> rotation droite.");
                turnSpot(turnVel, turnTime);
            }
        } else {
            // Orientation correcte -> on avance vers le dock
            System.out.println("[TRACK P1] Orientation OK -> avancer vers le dock.");
            move(fwdVel, moveTime);
        }
    }





    // Phase 2 du Track : caméra uniquement, pour orientation + petits pas vers le dock
    private void trackDockingApproach(double distToDock)
    {
        double score   = getTargetMaxScore();
        int    imgW    = getImageWidth();
        int    targetX = getTargetX();
        int    centerX = imgW / 2;
        int    dx      = targetX - centerX;

        long now = System.currentTimeMillis();

        System.out.println("[TRACK] Phase 2: approche caméra (dist ≈ "
                + String.format("%.2f", distToDock) + " m)");
        System.out.println("[TRACK] (score=" + String.format("%.2f", score)+ ")");

        // Seuils de qualité de détection du marqueur
        final double SCORE_MIN_SEARCH = 0.25; // "je vois quelque chose"
        final double SCORE_MIN_DOCK   = 0.42; // score exigé pour valider le dock

        // --- Scan initial : dès l'entrée en phase 2, on peut faire un tour pour chercher le marqueur ---
        if (!trackP2InitialScanDone) {

            // Si on voit déjà le marqueur correctement, pas besoin de spin
            if (score >= SCORE_MIN_SEARCH) {
                System.out.println("[TRACK] P2: marqueur déjà visible, pas de scan initial.");
                trackP2InitialScanDone = true;
            } else {
                System.out.println("[TRACK] P2: entrée en phase 2 -> scan initial à 360° pour trouver le marqueur.");
                float spinVel  = vel / 3.0f;
                int   spinTime = 1400;  // ~360°, ajuste si besoin

                turnSpot(spinVel, spinTime);

                // On mémorise qu'on a fait le scan, et on réinitialise la stabilisation
                trackP2InitialScanDone = true;
                dockStableSinceMs      = 0;
                trackP2LastSpinMs      = now;  // évite un spin "bloqué" juste après

                // On laisse la prochaine itération de trackDockingApproach analyser la situation
                return;
            }
        }

        // --- Gestion du cas "bloqué en droite-gauche" près du dock (inchangée) ---

        final long P2_SPIN_STUCK_MS    = 10_000L; // 10 s bloqué
        final long P2_SPIN_INTERVAL_MS = 10_000L; // 10 s

        if (!trackDocked && isStuck()) {
            if (trackP2StuckSinceMs == 0L) {
                // On vient de détecter qu'on est bloqué
                trackP2StuckSinceMs = now;
            }

            long stuckDuration = now - trackP2StuckSinceMs;
            long sinceLastSpin = now - trackP2LastSpinMs;

            if (stuckDuration > P2_SPIN_STUCK_MS && sinceLastSpin > P2_SPIN_INTERVAL_MS) {
                System.out.println("[TRACK] P2: bloqué près du dock, rotation complète pour retrouver le marqueur.");
                float spinVel  = vel / 3.0f;
                int   spinTime = 1400;  // ms, à ajuster si besoin

                turnSpot(spinVel, spinTime);

                dockStableSinceMs = 0;
                trackP2LastSpinMs = now;
                return;
            }
        } else {
            trackP2StuckSinceMs = 0L;
        }

        // --- le reste de ta fonction (deadBandOrient, deadBandDock, stabilisation...) reste inchangé ---

        // Bande morte "orientation" (phase d'approche)
        int deadBandOrient = imgW / 10;   // ±10% de la largeur

        // Bande morte "dock" : assez centré pour la condition finale
        int deadBandDock   = imgW / 14;

        // 1) Si le marqueur est quasi invisible -> rotation de recherche
        if (score < SCORE_MIN_SEARCH) {
            dockStableSinceMs = 0;
            System.out.println("[TRACK] P2: marqueur non vu (score="
                    + String.format("%.2f", score)
                    + "), rotation de recherche.");
            turnSpot(vel / 3.0f, 250);
            return;
        }

        // 2) Si on est encore plus loin que la distance cible...
        if (distToDock > TRACK_DOCKED_DIST) {
            if (Math.abs(dx) > deadBandOrient) {
                dockStableSinceMs = 0;
                float turnVel  = vel / 3.0f;
                int   turnTime = 150;

                if (dx > 0) {
                    System.out.println("[TRACK] P2: alignement -> marqueur à droite, rotation droite.");
                    turnSpot(turnVel, turnTime);
                } else {
                    System.out.println("[TRACK] P2: alignement -> marqueur à gauche, rotation gauche.");
                    turnSpot(-turnVel, turnTime);
                }
                return;
            }

            dockStableSinceMs = 0;
            System.out.println("[TRACK] P2: marqueur centré -> petit pas vers le dock.");
            move(vel * 0.5f, 200);
            return;
        }

        // 3) Zone de docking: distToDock <= TRACK_DOCKED_DIST
        boolean centeredFine = Math.abs(dx) < deadBandDock;
        boolean goodScore    = score >= SCORE_MIN_DOCK;

        if ((!centeredFine || !goodScore) && dockStableSinceMs == 0) {
            float turnVel  = vel / 4.0f;
            int   turnTime = 120;

            if (!centeredFine) {
                System.out.println("[TRACK] P2 (dock): recadrage fin (dx=" + dx + ").");
                if (dx > 0) {
                    turnSpot(turnVel, turnTime);
                } else {
                    turnSpot(-turnVel, turnTime);
                }
            } else {
                System.out.println("[TRACK] P2 (dock): score faible ("
                        + String.format("%.2f", score)
                        + "), micro rotation.");
                turnSpot(turnVel, turnTime);
            }
            return;
        }

        if (dockStableSinceMs == 0) {
            dockStableSinceMs = now;
            System.out.println("[TRACK] P2: pose dock bonne (dist ≤ 0.20, centré & score OK), on vérifie la stabilité...");
            setVel(0, 0);
            return;
        } else {
            long dt = now - dockStableSinceMs;
            if (dt > 600) {
                System.out.println("[TRACK] P2: Dock final validé. Arrêt.");
                setVel(0, 0);
                dir = 's';
                trackDocked = true;
                return;
            } else {
                System.out.println("[TRACK] P2: pose stable depuis " + dt + " ms, on attend encore.");
                setVel(0, 0);
                return;
            }
        }
    }




    public void avoid()
    {
        // Si tu as ce flag pour la phase 2 du docking, on le garde :
        if (trackDocked) {
            // Une fois docké, plus d'avoid
            setVel(0, 0);
            return;
        }

        // (Optionnel) Si tu as inTrackPhase2 :
        // Pendant le docking caméra, on laisse Track gérer l'approche
        // if (inTrackPhase2) return;

        // 1) Lecture de tous les sonars
        double s0 = getSonarRange(0); // gauche latéral
        double s1 = getSonarRange(1); // gauche diagonal
        double s2 = getSonarRange(2); // avant-gauche
        double s3 = getSonarRange(3); // avant-droite
        double s4 = getSonarRange(4); // droite diagonal
        double s5 = getSonarRange(5); // droite latéral

        // "Vraiment devant" : uniquement les deux capteurs quasi frontaux
        double frontMin = Math.min(s2, s3);

        // Pour choisir gauche/droite, on regarde les cônes "avant" (1,2) et (3,4)
        double leftFront  = Math.min(s1, s2); // gauche devant
        double rightFront = Math.min(s3, s4); // droite devant

        // Pour détecter un piège très proche, on regarde TOUT
        double globalMin = Math.min(
                Math.min(Math.min(s0, s1), Math.min(s2, s3)),
                Math.min(s4, s5)
        );

        // Seuil "il y a vraiment quelque chose devant"
        final double THRESH_FRONT = 0.45; // m
        // Seuil "je suis collé à un truc" → piège possible
        final double THRESH_TRAP  = 0.18; // m

        // 2) Si rien de vraiment proche devant ET pas de piège global, on ne fait rien
        //    -> permet de longer un mur sur le côté sans déclencher avoid
        if (frontMin > THRESH_FRONT && globalMin > THRESH_TRAP) {
            avoidStuckCounter = 0;
            return;
        }

        // 3) Détection d'un avoid qui tourne en rond / ne fait plus avancer le robot
        //    -> on utilise isStuck() qui regarde le GPS sur plusieurs secondes
        if (isStuck()) {
            avoidStuckCounter++;
        } else {
            avoidStuckCounter = 0;
        }

        // 4) Cas "piège" : très proche quelque part + coincé depuis un moment
        if (avoidStuckCounter >= 2 && globalMin < THRESH_TRAP) {
            System.out.println("[AVOID] Coincé près d'un obstacle -> manœuvre d'évasion.");

            // gros recul
            move(-vel * 0.7f, 600);

            // grande rotation aléatoire
            if (Math.random() < 0.5) {
                turnSpot(vel * 0.7f, 800);
            } else {
                turnSpot(-vel * 0.7f, 800);
            }

            avoidStuckCounter = 0;
            return;
        }


        // 5) Cas normal : obstacle vraiment devant, mais pas encore considéré “piégé”
        if (frontMin <= THRESH_FRONT) {
            System.out.println("[AVOID] Obstacle devant -> recul + rotation côté dégagé.");

            // petit recul pour se décoller du mur/meuble
            move(-vel * 0.5f, 250);

            // Choix du côté de rotation :
            //  diff > 0 => rightFront > leftFront => plus d'espace à droite
            double diff = rightFront - leftFront;

            // Si les deux côtés sont similaires, on choisit un côté au hasard
            if (Math.abs(diff) < 0.05) {
                diff = (Math.random() < 0.5) ? 1.0 : -1.0;
            }

            float turnVel  = vel / 2.0f;
            int   turnTime = 300;

            if (diff > 0) {
                // plus d'espace à droite -> tourne à droite
                turnSpot(turnVel, turnTime);
            } else {
                // plus d'espace à gauche -> tourne à gauche
                turnSpot(-turnVel, turnTime);
            }
        }
    }


    // Mise à jour de l'orientation estimée à partir des encodeurs
    private void updateOdometry() {
        double leftEnc  = getLeftWheelEnc();  // en tours
        double rightEnc = getRightWheelEnc(); // en tours

        if (!odoInit) {
            lastLeftEnc  = leftEnc;
            lastRightEnc = rightEnc;
            odoTheta     = 0.0;
            odoInit      = true;
            return;
        }

        double dLeftRev  = leftEnc  - lastLeftEnc;
        double dRightRev = rightEnc - lastRightEnc;

        lastLeftEnc  = leftEnc;
        lastRightEnc = rightEnc;

        // distance linéaire parcourue par chaque roue (approx), en mètres
        double dLeft  = 2.0 * Math.PI * WHEEL_RADIUS * dLeftRev;
        double dRight = 2.0 * Math.PI * WHEEL_RADIUS * dRightRev;

        // variation d'angle du robot (modèle diff-drive)
        double dTheta = (dRight - dLeft) / WHEEL_BASE;

        odoTheta = normalizeAngle(odoTheta + dTheta);
    }

    // Normalise un angle en radians dans [-pi, +pi]
    private double normalizeAngle(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
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
        // Sécurité : mêmes tailles, pas de null
        if (w_vec == null || s_vec == null || w_vec.length != s_vec.length) {
            // Ici on peut décider de renvoyer false ou de throw une exception.
            // Pour un projet étudiant, un false + message est souvent suffisant.
            System.err.println("TLU error: weight and sensor vectors must be non-null and of same length.");
            return false;
        }

        double sum = 0.0;

        // Somme pondérée Σ w_i * s_i
        for (int i = 0; i < w_vec.length; i++) {
            sum += w_vec[i] * s_vec[i];
        }

        // Activation si sum > f (comme dans l’énoncé)
        return (sum > f);
    }

    // Clamp simple dans [0,1]
    private double clamp01(double x) {
        if (x < 0.0) return 0.0;
        if (x > 1.0) return 1.0;
        return x;
    }

    // Affiche un message seulement si le comportement a changé
    private void logBehaviorChangeIfNeeded(int newBehavior) {
        if (newBehavior == currentBehavior) {
            return; // pas de changement, pas de log
        }

        String oldName;
        switch (currentBehavior) {
            case BEH_AVOID:  oldName = "AVOID";  break;
            case BEH_TRACK:  oldName = "TRACK";  break;
            case BEH_CLEAN:  oldName = "CLEAN";  break;
            case BEH_WANDER: oldName = "WANDER"; break;
            default:         oldName = "NONE";   break;
        }

        String newName;
        switch (newBehavior) {
            case BEH_AVOID:  newName = "AVOID";  break;
            case BEH_TRACK:  newName = "TRACK";  break;
            case BEH_CLEAN:  newName = "CLEAN";  break;
            case BEH_WANDER: newName = "WANDER"; break;
            default:         newName = "NONE";   break;
        }

        System.out.println("[AUTO] Comportement " + oldName + " -> " + newName);
        currentBehavior = newBehavior;
    }

    // Retourne true si le robot bouge très peu depuis plusieurs secondes
    private boolean isStuck() {
        long now = System.currentTimeMillis();

        // On ne teste qu'environ une fois par seconde
        if (now - lastStuckCheckTimeMs < 1000) {
            return false;
        }

        double x = getGPSX();
        double y = getGPSY();

        double dx = x - lastStuckCheckX;
        double dy = y - lastStuckCheckY;
        double dist = Math.sqrt(dx * dx + dy * dy);

        lastStuckCheckTimeMs = now;
        lastStuckCheckX = x;
        lastStuckCheckY = y;

        // Si on s’est déplacé de moins de 5 cm depuis la dernière seconde
        if (dist < 0.05) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }

        // Considérer “bloqué” après 5 secondes sans vraiment bouger
        return (stuckCounter >= 5);
    }

}
