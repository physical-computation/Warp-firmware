import processing.serial.*;
import org.gicentre.utils.stat.*;    // For chart classes.
import grafica.*;
import java.util.Random;


Serial devPort;
PFont titleFont,smallFont;

String serialInputBuffer = "";
String delims = "[,]+";

int nAvg = 15;
int visSpectBands = 6;
int nirSpectBands = 6;
int visSpectMaxY = 2000;

// float[] lastPos = new float[2];

BarChart VisSpectChart;
BarChart NIRSpectChart;

float camFPS = 0;
float[] LI_sense_data = new float[2];
// float[] dmdSpeeds = new float[2];
// float[] AvgSpeeds = new float[2];

float [] LI_sense1_data = new float[80];
float [] LI_sense2_data = new float[80];
float [] timeSeries = new float[80];
// float [] cameraFPS = new float [80];

XYChart LI_sense1_chart;
XYChart LI_sense2_chart;
XYChart expectedFPS;

float [] h_scores = new float[4];

// float[] visSpect_data = new float[] {2462,4490,6401,7657,9649,15154};
float[] visSpect_data = new float[] {0,0,0,0,0,0};
float[] AVG_visSpect_data = new float[] {0,0,0,0,0,0};
float[] nirSpect_data = new float[] {2462,4490,7657,9649,18200,23124};
String[] visSpect_labels = new String[] {"Violet","Blue","Green","Yellow","Orange", "Red"};
String[] nirSpect_labels = new String[] {"lambda_1","lambda_2","lambda_3","lambda_4","lambda_5", "lambda_6"};

void dataShiftBack(float stream1, float stream2){

  for (int i = 0; i < LI_sense1_data.length-1; i++) {
    LI_sense1_data[i] = LI_sense1_data[i+1];
    LI_sense2_data[i] = LI_sense2_data[i+1];
    // cameraFPS[i] = cameraFPS[i+1];
    timeSeries[i] = timeSeries[i+1];
  }
  LI_sense1_data[LI_sense1_data.length-1] = stream1;
  LI_sense2_data[LI_sense2_data.length-1] = stream2;
  // cameraFPS[cameraFPS.length-1] = camFPS;
  timeSeries[timeSeries.length-1] = timeSeries[timeSeries.length-2] + 1;
}


void setup() {
  size(100, 100);

  printArray(Serial.list());

  try{
    //devPort = new Serial(this, "/dev/ttyS0", 19200);
    devPort = new Serial(this, "COM8", 19200);
  } catch (IndexOutOfBoundsException e) {
    System.err.println("IndexOutOfBoundsException: " + e.getMessage());
  }

  String[] args = {"Multi Frame Test"};
  //USapplet US_sensors = new USapplet();
  
  LightIntensity_frame lightIntensityDisplay = new LightIntensity_frame();
  //OrientationApplet orientationDisplay = new OrientationApplet();
  //PositionApplet positionDisplay = new PositionApplet();
  VisSpectrum_frame visSpectrumDisplay = new VisSpectrum_frame();
  // NIRSpectrum_frame nirSpectrumDisplay = new NIRSpectrum_frame();
  //FPSApplet fpsDisplay = new FPSApplet();

  //  PApplet.runSketch(args, US_sensors);
  PApplet.runSketch(args, lightIntensityDisplay);
  // PApplet.runSketch(args, orientationDisplay);
  //PApplet.runSketch(args, positionDisplay);
  PApplet.runSketch(args, visSpectrumDisplay);
  // PApplet.runSketch(args, nirSpectrumDisplay);
  //PApplet.runSketch(args, fpsDisplay);

}


void draw() {
  background(0);
  try {
    while (devPort.available() > 0) {
      //println("running");
      serialInputBuffer = devPort.readStringUntil('\n');
      if (serialInputBuffer != null) {
        serialInputBuffer = serialInputBuffer.trim();
        println(serialInputBuffer);
        String[] dataTokens = serialInputBuffer.split(delims);
        println(dataTokens.length);
        if (dataTokens.length == 7){ //set the number of data objects expected

          //for (int i = 0; i < 7; i++) {
          //   distances[i+6] = int(Float.parseFloat(dataTokens[i+24]));
          //}



          // LI_sense_data[0] = Float.parseFloat(dataTokens[0]);
          // LI_sense_data[1] = Float.parseFloat(dataTokens[1]);

          for (int i = 0; i < visSpectBands; i++) {
            visSpect_data[i] = int(Float.parseFloat(dataTokens[i+1]));
          }
          // for (int i = 0; i < nirSpectBands; i++) {
          //   nirSpect_data[i] = int(Float.parseFloat(dataTokens[i+2]));
          // }

          // camFPS = Float.parseFloat(dataTokens[0]);

          // dataShiftBack(LI_sense_data[0], LI_sense_data[1]);
          // LI_sense1_chart.setMinX(min(timeSeries));
          // LI_sense1_chart.setMaxX(max(timeSeries));
          // LI_sense1_chart.setData(timeSeries, LI_sense1_data);

          // LI_sense2_chart.setMinX(min(timeSeries));
          // LI_sense2_chart.setMaxX(max(timeSeries));
          // LI_sense2_chart.setData(timeSeries, LI_sense2_data);

          //targetLocation[0] = Float.parseFloat(dataTokens[11]);
          //targetLocation[1] = Float.parseFloat(dataTokens[12]);
          //porterLocation_Global[0] = Float.parseFloat(dataTokens[13]);
          //porterLocation_Global[1] = Float.parseFloat(dataTokens[14]);

          //porterOrientation = degrees(Float.parseFloat(dataTokens[17]));
          //wheelOrientation = degrees(Float.parseFloat(dataTokens[23]));
          //imuOrientation = degrees(Float.parseFloat(dataTokens[18]));

          //for (int i = 0; i < 4; i++) {
          //   h_scores[i] = int(Float.parseFloat(dataTokens[i+31]));
          //}
        }
      }
    }
  } catch (NullPointerException e){
    System.err.println("NullPointerException: " + e.getMessage());
    LI_sense_data[0] = 0;
    LI_sense_data[1] = 0;
    // camFPS = 0;
  }
}

public class LightIntensity_frame extends PApplet {
  int tCount = 0;

  public void settings(){
    size(800,300);
  }
  public void setup() {

    surface.setLocation(0, 0);

    LI_sense1_chart = new XYChart(this);
    LI_sense2_chart = new XYChart(this);

    LI_sense2_chart.setData(timeSeries, LI_sense2_data);
    LI_sense1_chart.setData(timeSeries, LI_sense1_data);

    // Axis formatting and labels.
    LI_sense1_chart.showXAxis(true);
    LI_sense1_chart.showYAxis(true);

    LI_sense1_chart.setMinY(-30);
    LI_sense2_chart.setMinY(-30);

    LI_sense1_chart.setMaxY(2000);
    LI_sense2_chart.setMaxY(2000);

    LI_sense1_chart.setYFormat("###");  // M
    LI_sense1_chart.setXFormat("0000");      // Time

    LI_sense2_chart.setYFormat("###");  //
    LI_sense2_chart.setXFormat("0000");      // Time

    // Symbol colours
    LI_sense1_chart.setPointColour(color(180,50,50,100));
    LI_sense1_chart.setPointSize(5);
    LI_sense1_chart.setLineWidth(1);

    LI_sense2_chart.setPointColour(color(50,50,180,180));
    LI_sense2_chart.setPointSize(5);
    LI_sense2_chart.setLineWidth(1);
  }

  public void draw() {
    background(255);
    textSize(12);
    try {
      LI_sense1_chart.draw(15,15,width-30,height-30);
      //LI_sense2_chart.draw(37,23,width-67,height-53);
    } catch (NullPointerException e){
    }

    //Draw a title over the top of the chart.
    fill(120);
    textSize(20);
    text("Light Intensity", 70,30);
    textSize(11);
    text("(unnormalised raw measurements)", 70,45);
  }
}

public class VisSpectrum_frame extends PApplet {

    void mAverage(int n) {
    int i = 0;
    for (i = 0; i < visSpect_data.length; i++) {
      AVG_visSpect_data[i] = AVG_visSpect_data[i] + (visSpect_data[i] - AVG_visSpect_data[i])/n;
    }
  }

  public void settings(){
      size(800,400);
  }

  public void setup(){
    surface.setLocation(0, 360);

    smooth();
    // noLoop();
    
    // titleFont = loadFont("Helvetica-22.vlw");
    // smallFont = loadFont("Helvetica-12.vlw");
    // textFont(smallFont);

    VisSpectChart = new BarChart(this);
    VisSpectChart.setData(AVG_visSpect_data);
    VisSpectChart.setBarLabels(visSpect_labels);
    VisSpectChart.setMaxValue(visSpectMaxY);
    VisSpectChart.setBarColour(color(200,80,80,100));
    VisSpectChart.setBarGap(2); 
    VisSpectChart.setValueFormat("###,###");
    VisSpectChart.showValueAxis(true); 
    VisSpectChart.showCategoryAxis(true); 
  }


  public void draw(){
    background(255);
    if (visSpectMaxY < int(max(visSpect_data))) {
      visSpectMaxY = int(max(visSpect_data));
    }
    VisSpectChart.setMaxValue(visSpectMaxY);

    // VisSpectChart.setData(visSpect_data);
    // VisSpectChart.setBarLabels(visSpect_labels);

    // VisSpectChart.setBarColour(color(200,80,80,100));
    // VisSpectChart.setBarGap(2); 
    // VisSpectChart.setValueFormat("$###,###");
    // VisSpectChart.showValueAxis(true); 
    // VisSpectChart.showCategoryAxis(true);
    println(visSpect_data);
    mAverage(3);

    VisSpectChart.draw(30,30,width-40,height-40);
    // VisSpectChart.updateLayout();

    fill(120);
    // textFont(titleFont);
    text("Relative Intensity", 70,30);
    float textHeight = textAscent();
    // textFont(smallFont);
    text("Wavelength Band (Visible)", 70,30+textHeight);
  }
}

public class NIRSpectrum_frame extends PApplet {
  public void settings(){
      size(800,300);
  }

  public void setup(){
    surface.setLocation(0, 700);

    smooth();
    // noLoop();
    
    // titleFont = loadFont("Helvetica-22.vlw");
    // smallFont = loadFont("Helvetica-12.vlw");
    // textFont(smallFont);

    NIRSpectChart = new BarChart(this);
    NIRSpectChart.setData(nirSpect_data);
    NIRSpectChart.setBarLabels(nirSpect_labels);

    NIRSpectChart.setBarColour(color(200,80,80,100));
    NIRSpectChart.setBarGap(2); 
    NIRSpectChart.setValueFormat("$###,###");
    NIRSpectChart.showValueAxis(true); 
    NIRSpectChart.showCategoryAxis(true); 
  }


  public void draw(){
    background(255);
    
    NIRSpectChart.draw(10,10,width-20,height-20);
    fill(120);
    // textFont(titleFont);
    text("Relative Intensity", 70,30);
    float textHeight = textAscent();
    // textFont(smallFont);
    text("Wavelength Band (NIR)", 70,30+textHeight);
  }
}
