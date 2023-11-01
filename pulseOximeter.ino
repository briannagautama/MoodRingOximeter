/*-------------------------------------------------------------------------------------------

  Develop a pulse oximeter using Arduino DUE with the following functionalities:
  1. Real-time signal sampling of PPG signal
  2. Waveform display on LCD Display
  3. Waveform filtering
  4. Stable heart rate and SpO2 level on LCD display

---------------------------------------------------------------------------------------------*/



//==========================================================================================//
//                                1. Variable declaration                                   //
//==========================================================================================//

//================================= Pulse oximeter ===================================//

#include <Adafruit_ILI9340.h>

#define dc 53
#define cs 45
#define rst 49

Adafruit_ILI9340 TFT_LCD = Adafruit_ILI9340 (cs, dc, rst);


bool signal_gen = true;                // ON/OFF of signal generation
float signal_freq = 50;                // Signal frequency (Hz)
float signal_period = (1/signal_freq)*1000000  ;  
int num_outputch = 2;                // Number of output channel (analog ouput, starting from DAC0)
float signal_amp = 0.5;               // Signal amplitude (V)
float signal_offset = 1;                // Signal offset (V)
float max_v_out = 3.3;                // Max. voltage output (V)
int output, output0, output1, output_smooth;               // Output signal (digital)
int max_bit_val = pow(2,12);
float output_val;
int pd_rise_fall = 250;

bool sig_detection = true;                // ON/OFF of signal detection
int input, input0, input1;                // Input reading (digital)
int red_start, ir_start, read_red, read_ir;
double red_1, red_2, ir_1, ir_2;
int red_t1, red_t2, ir_t1, ir_t2;
double old_ir_min=0,old_ir_max=4095,old_red_min=0,old_red_max=4095,ir_min,ir_max,red_min,red_max;



//================================= Filtering ===================================//

// Filter order //
const int filter_order = 20;

// Temporal storage array for the past readings //
double input_red[filter_order+1];
double input_ir[filter_order+1];
int count=0;
double filtered_red, filtered_ir;

                
// Filtering coeffieient //

// 100 Hz, 10 Hz cutoff
double filter_coefficients[21] = {-6.21115458751030e-19,  -0.00212227114882539,  -0.00632535399151418,  -0.0116118103776210, -0.0123546567489824, 4.19252934656945e-18,
                                  0.0317744975585673,  0.0814359075642177,  0.137493781701943, 0.182125490388735, 0.199168830106960, 0.182125490388735, 0.137493781701943,
                                  0.0814359075642177,  0.0317744975585673,  4.19252934656945e-18,  -0.0123546567489824, -0.0116118103776210, -0.00632535399151418,
                                  -0.00212227114882539,  -6.21115458751030e-19};




//Smoothing
const int window = 10;
double presmoothed_red[window], presmoothed_ir[window];
double smoothed_red[200], smoothed_ir[200];
int smoothing_count=0;
double smoothed_out_ir, smoothed_out_red;
double start_time = 0, end_time = 0;

bool pulse_calculated = false;
int pulse_rate_arr[20]={0};
int pulse_counter = 0;
int increasing_count=0;
int pulse_rate = 0;
int final_mean_pulse=0, new_pulse_rate=0;
double r_val, deriv_red, deriv_ir;
double old_r_val=0;

//==========================================================================================//
//                            2. Main program (initialize environment)                      //
//==========================================================================================//
void setup() {

  // Turn on serial port (baud, bit per second) //
  Serial.begin(57600);

  // Set analog read & write resolution (bit) //
  analogReadResolution(12);
  analogWriteResolution(12);

  TFT_LCD.begin();
  TFT_LCD.fillScreen(ILI9340_BLACK);
  TFT_LCD.setCursor(5, 10);
  TFT_LCD.setTextSize(2);
  TFT_LCD.setRotation(1);
  TFT_LCD.print("Pulse Rate");  
  TFT_LCD.setCursor(190, 10);
  TFT_LCD.print("SpO2");  
}



//==========================================================================================//
//                                 3. Main program (looping)                                //
//==========================================================================================//
void loop() {
  
  // Sampling and operating the LEDs and photodiode //
  if (signal_gen == true) {   // ON/OFF of signal generation 
    for (int i=0; i<num_outputch; i++) {    // Two signal generation channels by looping 
      output_val = 1;
      
      // Limit the output signal level to be within availiable output range of the board //
      output = int((((output_val * signal_amp) + signal_offset)/max_v_out) *max_bit_val);
      if ((output >= 0) and (output <= max_bit_val)){
        output = output;
      } else  if (output>max_bit_val){
        output = max_bit_val;
      } else if (output < 0){
        output = 0;
      }
      
      // Output waveform //
      if (i==0) {
        output0 = output;
        digitalWrite(11, HIGH);    // output at D2
        red_start = micros();
        if (sig_detection == true) {   // ON/OFF of signal detection     
          while ((micros()-red_start)<pd_rise_fall){
            digitalWrite(11, HIGH);    // output at DAC0   
          }
          // Sampling at analog channel defined above //
          input0 = (max_bit_val - analogRead(A0)); 
          read_red = micros();
          Serial.print(input0);
          Serial.print("\t");

          filtered_red = myfilter(filter_coefficients, filter_order, 0, input0, count);
          Serial.print(filtered_red);
          Serial.print("\t");
          if (count%5 == 0){
              r_val = calc_r_val(filtered_red, read_red, filtered_ir, read_ir);
          }

        delayMicroseconds((signal_period/2)-(micros()-red_start));
        output0 = 0;
        digitalWrite(11, LOW); 
        
        // delay to account for photodiode rise/fall time
        delayMicroseconds(pd_rise_fall+100);
        }
      }
      
      else if (i==1) {
        output1 = output;
        analogWrite(DAC1, output1);    // output at DAC1
        ir_start = micros();
        if (sig_detection == true) {   // ON/OFF of signal detection
          while ((micros()-ir_start)<pd_rise_fall){
            analogWrite(DAC1, output1);    // output at DAC1  
          }
          
          // Sampling at analog channel defined above //
          input1 = (max_bit_val - analogRead(A0)); 
          read_ir = micros();
          Serial.print(input1); 
          Serial.print("\t"); 
          filtered_ir = myfilter(filter_coefficients, filter_order, 1, input1, count);
          Serial.print(filtered_ir);
          Serial.print("\t"); 
          smoothed_out_ir = smoothing(filtered_ir);
                
          if (count==0){
            old_ir_min = filtered_ir;
            old_ir_max = filtered_ir;
            old_red_min = filtered_red;
            old_red_max = filtered_red;
          }else if (count%320 == 0){
            ir_min = old_ir_min - 150;
            ir_max = old_ir_max + 150;
            red_min = old_red_min - 75;
            red_max = old_red_max + 75;
            old_ir_min = filtered_ir;
            old_ir_max = filtered_ir;
            old_red_min = filtered_red;
            old_red_max = filtered_red;
          }
          if (filtered_ir < ir_min){
            old_ir_min = filtered_ir;
          }
          if (filtered_ir > ir_max){
            old_ir_max = filtered_ir;
          }
          if (filtered_red < red_min){
            old_red_min = filtered_red;
          }
          if (filtered_red > red_max){
            old_red_max = filtered_red;
          }
          
          float y_ir = int(map(filtered_ir, ir_min, ir_max, 240, 160));
          float y_red = int(map(filtered_red, red_min, red_max, 160, 80));
          int x_coordinate = count % 320;

          // dynamic printing of PPG waveforms (red and IR) on LCD
          TFT_LCD.fillRect(x_coordinate,80,5,160, ILI9340_BLACK);
          if (y_red >=80 && y_red <= 160){
            TFT_LCD.writePixel(x_coordinate, y_red, ILI9340_WHITE);
          }
          if (y_ir >=160 && y_ir <=240){
            TFT_LCD.writePixel(x_coordinate, y_ir, ILI9340_WHITE);
          }
                
        }
        delayMicroseconds((signal_period/2)-(micros()-ir_start));
        output1 = 0;
        analogWrite(DAC1, output1); 
        delayMicroseconds(pd_rise_fall+100);
      }
    }
    count++;
    
    
    
  }
}


//==========================================================================================//
//                                      4. Filtering                                        //
//==========================================================================================//
double myfilter (double filter_coefficients[ ], int filter_order, int type, int curr_input, int count)  {    
  /*
   * Filter PPG waveforms
   */
  if (type == 0){
    // Filter PPG waveform corresponding to red LED
    
    if (count<(filter_order+1)){   
      // Filtering while running-in (before moving average window is filled) //
 
      // Define output variables //
      input_red[count] = curr_input;
  
      // Calculate output variables //
      output = curr_input;
  
      // return output variables //
      return output;
  
    } else {    // Filtering after running-in //

      // Define output variables //
      output = 0;
  
      // Calculate output variable //
      for (int j=0; j<filter_order; j++){
        input_red[j] = input_red[j+1];
      }
      input_red[filter_order] = curr_input;
      for (int j=0; j<filter_order+1; j++){
        output += filter_coefficients[j] * input_red[filter_order-j];
      }
      
      // return output variable //
      return output;
    }
    
  } else if (type == 1){
    // Filter PPG waveform corresponding to IR LED
      
    if (count<(filter_order+1)){   
      // Filtering while running-in //
      
      // Define output variables //
      input_ir[count] = curr_input;
  
      // Calculate output variables //
      output = curr_input;
  
      // return output variables //
      return output;

    } else {    
      // Filtering after running-in //
      // Define output variables //
      output = 0;
  
      // Calculate output variable //
      for (int j=0; j<filter_order; j++){
        input_ir[j] = input_ir[j+1];
      }
      input_ir[filter_order] = curr_input;
      for (int j=0; j<filter_order+1; j++){
        output += filter_coefficients[j] * input_ir[filter_order-j];
      }
      
      // return output variable //
      return output;
    }
  }
}


//==========================================================================================//
//                                      5. Smoothing                                        //
//==========================================================================================//

double smoothing(double curr_input){
  /*
   * Smoothing of PPG waveform
   */
  if (count<(window)){
      presmoothed_ir[count] = curr_input;
      return curr_input;
  
   } else {    
      // Define output variables //
      output_smooth = 0;
      
      // Calculate output variable //
      for (int j=0; j<window-1; j++){
        presmoothed_ir[j] = presmoothed_ir[j+1];
      }
      presmoothed_ir[window-1] = curr_input;
      for (int j=0; j<window; j++){
        output_smooth = output_smooth + presmoothed_ir[j];
      }
      
      smoothing_count = smoothing_count+1;
      smoothed_ir[smoothing_count] = output_smooth/window;
      
      if ((smoothed_ir[smoothing_count]>=smoothed_ir[smoothing_count-1])){
        increasing_count++;
      } else if (smoothed_ir[smoothing_count]<smoothed_ir[smoothing_count-1]) {
        increasing_count = 0;
      } 
      
      if (smoothing_count >= 199){
        double temp = smoothed_ir[199];
        smoothed_ir[0] = temp;
        smoothing_count = 0;
      }
      if (increasing_count == 5){
        
        if (start_time == 0){
            start_time = millis();
          } else {
            end_time = millis();
            calc_pulse_rate(start_time, end_time); //here
            pulse_calculated = true;
            start_time = end_time;
          }
      }
      return output_smooth/window;
    }
}


//==========================================================================================//
//                                      6. Pulse rate                                       //
//==========================================================================================//
void calc_pulse_rate(double start_time, double end_time){
  /*
   * Calculate pulse rate from detected start of peak and detected end of peak
   * Display pulse rate on LCD Display
   */
  new_pulse_rate = floor((60000) / (end_time-start_time));
  
  if (new_pulse_rate <20 || new_pulse_rate > 200){
    return;
  }
  
  if (pulse_counter < 14){
    pulse_counter++;
  } 
  int mean_pulse=0;
  if (pulse_counter == 14){
    for (int s=0; s<pulse_counter;s++){
      pulse_rate_arr[s] = pulse_rate_arr[s+1];
    }
  }
  pulse_rate_arr[pulse_counter] = new_pulse_rate;
  for (int r=0; r<pulse_counter;r++){
    mean_pulse += pulse_rate_arr[r];
  }
  mean_pulse = mean_pulse/pulse_counter;
  final_mean_pulse=0;
  int idx = 0;
  for (int r=0; r<pulse_counter;r++){
    if (abs(pulse_rate_arr[r]-mean_pulse) < 30) {
      final_mean_pulse += pulse_rate_arr[r];
      idx++;
    }
  }

  final_mean_pulse = final_mean_pulse/idx;
  if (final_mean_pulse!=pulse_rate){
    pulse_rate = final_mean_pulse;
    TFT_LCD.fillRect(5,40,80,40, ILI9340_BLACK);
    TFT_LCD.setCursor(5, 40);
   
    TFT_LCD.setTextColor(ILI9340_WHITE);
    TFT_LCD.print(pulse_rate);
  }
}

//==========================================================================================//
//                                   7. SpO2 Calculation                                    //
//==========================================================================================//
double calc_r_val(double filtered_red, int read_red, double filtered_ir, int read_ir){
  /*
   * Calculate R value/SpO2 level from PPG waveform
   */
  if (count > 20){
    if (int(count/5) == 1){
      red_1 = filtered_red;
      red_t1 = read_red;  
      ir_1 = filtered_ir;
      ir_t1 = read_ir;
      return 0;
    } else{
      red_2 = filtered_red;
      red_t2 = read_red;
      ir_2 = filtered_ir;
      ir_t2 = read_ir;
    }
    Serial.print("Calculate r Value: dred=");
    deriv_red = (red_2 - red_1)/(red_t2 - red_t1);
    Serial.print(deriv_red);
    Serial.print(" dir= ");
    deriv_ir = (ir_2 - ir_1)/(ir_t2 - ir_t1);
    Serial.println(deriv_ir);
    double r_val = deriv_red/deriv_ir;
    Serial.println(r_val);
    if (old_r_val == 0 || old_r_val!=r_val){
      TFT_LCD.fillRect(190,40,80,40, ILI9340_BLACK);
      TFT_LCD.setCursor(190, 40);
      TFT_LCD.print(r_val);
    }
    
    old_r_val = r_val;
    red_1 = red_2;
    red_t1 = red_t2;
    ir_1 = ir_2;
    ir_t1 = ir_t2;
    return r_val;
    
  }
}
