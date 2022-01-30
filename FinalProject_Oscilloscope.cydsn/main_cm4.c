//Lillian C. Gwendolyn
//CSE121/L
//final project :3

#include "project.h"
#include "GUI.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

#include "string.h"

//GUI defines
#define LCD_XSIZE 320 // Width of LCD screen in pixels
#define LCD_YSIZE 240 // Height of LCF screen in pixels
#define GRID_XDIVISIONS 10 // Number of x-axis divisions for grid
#define GRID_YDIVISIONS 8 // Number of y-axis divisions for grid
#define LCD_XMARGIN 5 // Margin around screen on x-axis
#define LCD_YMARGIN 4 // Margin around screen on y-axis
#define MAXPOINTS (LCD_XSIZE - (2 * LCD_XMARGIN)) // Maximum number of points in wave for plotting
#define MAYPOINTS (LCD_YSIZE - (2 * LCD_YMARGIN))

#define DISPLAY_FREQ_XPOS 200
#define DISPLAY_FREQ_YPOS 15

#define DISPLAY_SCALE_XPOS 10
#define DISPLAY_SCALE_YPOS 15

#define PI 3.14159265

//other defines
#define ADC_MAX 4096
#define ADC_FREQUENCY 250000
#define MAX_VOLTAGE 3300

#define CHANNEL_DATA_SIZE 2048

#define PIX_PER_XDIV (MAXPOINTS / 10)
#define PIX_PER_YDIV ((LCD_YSIZE - (2 * LCD_YMARGIN)) / 8)

//UART communication defines
#define UART_IN_STR_START "start"
#define UART_OUT_STR_START "TinyScope started\r\n"

#define UART_IN_STR_STOP "stop"
#define UART_OUT_STR_STOP "TinyScope stopped\r\n"

#define UART_IN_STR_SET_MODE_FREE "set mode free"
#define UART_OUT_STR_SET_MODE_FREE "Mode set to free-running\r\n"

#define UART_IN_STR_SET_MODE_TRIG "set mode trigger"
#define UART_OUT_STR_SET_MODE_TRIG "Mode set to trigger\r\n"

#define UART_IN_STR_SET_TRIG_LEVEL "set trigger_level "
#define UART_OUT_STR_SET_TRIG_LEVEL "Trigger level set to "
#define UART_IN_STR_SET_TRIG_LEVEL_SIZE 18

#define UART_IN_STR_SET_TRIG_SLOPE_POS "set trigger_slope positive"
#define UART_OUT_STR_SET_TRIG_SLOPE_POS "Trigger slope set to positive\r\n"

#define UART_IN_STR_SET_TRIG_SLOPE_NEG "set trigger_slope negative"
#define UART_OUT_STR_SET_TRIG_SLOPE_NEG "Trigger slope set to negative\r\n"

#define UART_IN_STR_SET_TRIG_CHANNEL_ONE "set trigger_channel 1"
#define UART_OUT_STR_SET_TRIG_CHANNEL_ONE "Trigger channel set to 1\r\n"

#define UART_IN_STR_SET_TRIG_CHANNEL_TWO "set trigger_channel 2"
#define UART_OUT_STR_SET_TRIG_CHANNEL_TWO "Trigger channel set to 2\r\n"

#define UART_IN_STR_SET_XSCALE "set xscale "
#define UART_OUT_STR_SET_XSCALE "xscale set to "
#define UART_IN_STR_SET_XSCALE_SIZE 11

#define UART_IN_STR_SET_YSCALE "set yscale "
#define UART_OUT_STR_SET_YSCALE "yscale set to "
#define UART_IN_STR_SET_YSCALE_SIZE 11

#define UART_OUT_STR_INPUT_ERR "ERR: UNSUPPORTED UART INPUT\r\n"
#define UART_OUT_STR_RUNNING_ERR "ERR: STOP TINYSCOPE FIRST\r\n"


//used for drawing
uint16_t ch1_currydraw[MAXPOINTS];
uint16_t ch2_currydraw[MAXPOINTS];

uint16_t ch1_prevydraw[MAXPOINTS];
uint16_t ch2_prevydraw[MAXPOINTS];

uint16_t ch1_currpot = 0;
uint16_t ch2_currpot = 0;

uint16_t ch1_prevpot = 0;
uint16_t ch2_prevpot = 0;

//UART info
char UART_RX_buffer[100];
uint8_t UART_RX_buffer_pos = 0;

//DMA info
uint16_t ch1_newdata[CHANNEL_DATA_SIZE];
uint16_t ch2_newdata[CHANNEL_DATA_SIZE];

uint16_t ch1_currdata[CHANNEL_DATA_SIZE];
uint16_t ch2_currdata[CHANNEL_DATA_SIZE];

/* //smoothed data actually results in worse accuracy, left in for propriety
uint16_t ch1_smoothdata[CHANNEL_DATA_SIZE];
uint16_t ch2_smoothdata[CHANNEL_DATA_SIZE];
#define SMOOTH_DATA_START 4
*/

uint8_t ch1_dma_desc_position = 0;
uint8_t ch2_dma_desc_position = 0;

//state stuff
bool isRunning = false;
bool hasPrintedBackground = false;

enum RUN_MODE_ENUM {freerunning, trigger}RunMode = freerunning;

enum TRIG_SLOPE_ENUM {pos = 1, neg = -1}TrigSlope = pos;

uint8_t TrigChan = 1;
uint16_t TrigLevel = 0;
uint16_t xscale = 1000;
uint16_t yscale = 1000;

uint16_t ch1_freq = 1;
uint16_t ch2_freq = 1;

uint16_t ch1_trigger_start = 0;
uint16_t ch2_trigger_start = 0;

//function to handle UART I/O
void manageUART(void){
    
    bool statement_complete = false;
    
    while(UART_GetNumInRxFifo() > 0){
        
        //get char that was placed in
        UART_RX_buffer[UART_RX_buffer_pos] = getchar();
        //put char on tx fifo so that the person can see what was printed
        printf("%c", UART_RX_buffer[UART_RX_buffer_pos]);
        
        if(UART_RX_buffer[UART_RX_buffer_pos] == '\n'){
            
            statement_complete = true;
            UART_RX_buffer[UART_RX_buffer_pos] = '\0';
            UART_RX_buffer_pos = 0;
            break;//and hope that there isnt anything else here to worry about
        }
        
        //go to next buffer position
        ++UART_RX_buffer_pos;
    }
    
    //clear status after receiving all data
    UART_ClearRxFifoStatus(CY_SCB_UART_RX_NOT_EMPTY);
    
    //if statement not complete then nothing else to handle, return
    if(!statement_complete) return;
    
    //cant do switch statement w strcmp :sob:
    if(!(strncmp(UART_RX_buffer, UART_IN_STR_START, sizeof(UART_IN_STR_START)))){
        
        isRunning = true;
        hasPrintedBackground = false;
        
        UART_PutString(UART_OUT_STR_START);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_STOP, sizeof(UART_IN_STR_STOP)))){
        
        isRunning = false;
        hasPrintedBackground = false;
        
        UART_PutString(UART_OUT_STR_STOP);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_MODE_FREE, sizeof(UART_IN_STR_SET_MODE_FREE)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        RunMode = freerunning;
        
        UART_PutString(UART_OUT_STR_SET_MODE_FREE);
    
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_MODE_TRIG, sizeof(UART_IN_STR_SET_MODE_TRIG)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        RunMode = trigger;
        
        UART_PutString(UART_OUT_STR_SET_MODE_TRIG);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_TRIG_LEVEL, UART_IN_STR_SET_TRIG_LEVEL_SIZE))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        //get level
        //stops at first non int char
        int newLevel = atoi(UART_RX_buffer + UART_IN_STR_SET_TRIG_LEVEL_SIZE);
        
        if((newLevel <= 3000) && (newLevel >= 0) && (newLevel == ((newLevel / 100) * 100))){
            char str[10];
            
            UART_PutString(UART_OUT_STR_SET_TRIG_LEVEL);
            sprintf(str, "%d mV\r\n", newLevel);
            UART_PutString(str);
            TrigLevel = newLevel;
            
        }else UART_PutString(UART_OUT_STR_INPUT_ERR);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_TRIG_SLOPE_POS, sizeof(UART_IN_STR_SET_TRIG_SLOPE_POS)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        TrigSlope = pos;
        
        UART_PutString(UART_OUT_STR_SET_TRIG_SLOPE_POS);
        
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_TRIG_SLOPE_NEG, sizeof(UART_IN_STR_SET_TRIG_SLOPE_NEG)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        TrigSlope = neg;
        
        UART_PutString(UART_OUT_STR_SET_TRIG_SLOPE_NEG);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_TRIG_CHANNEL_ONE, sizeof(UART_IN_STR_SET_TRIG_CHANNEL_ONE)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        TrigChan = 1;
        
        UART_PutString(UART_OUT_STR_SET_TRIG_CHANNEL_ONE);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_TRIG_CHANNEL_TWO, sizeof(UART_IN_STR_SET_TRIG_CHANNEL_TWO)))){
        
        //cannot be entered while running
        if(isRunning){
            
            UART_PutString(UART_OUT_STR_RUNNING_ERR);
            return;
            
        }
        
        TrigChan = 2;
        
        UART_PutString(UART_OUT_STR_SET_TRIG_CHANNEL_TWO);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_XSCALE, UART_IN_STR_SET_XSCALE_SIZE))){
        
        //get xscale
        //stops at first non int char
        int newScale = atoi(UART_RX_buffer + UART_IN_STR_SET_XSCALE_SIZE);
        
        //doesnt limit values quite as much as needed but its good enough
        if((newScale <= 10000) && (newScale >= 0) && (newScale == ((newScale / 100) * 100))){
            char str[11];
            
            UART_PutString(UART_OUT_STR_SET_XSCALE);
            sprintf(str, "%d us\r\n", newScale);
            UART_PutString(str);
            xscale = newScale;
            
        }else UART_PutString(UART_OUT_STR_INPUT_ERR);
        
    }else if(!(strncmp(UART_RX_buffer, UART_IN_STR_SET_YSCALE, UART_IN_STR_SET_YSCALE_SIZE))){
        
        //get yscale
        //stops at first non int char
        int newScale = atoi(UART_RX_buffer + UART_IN_STR_SET_YSCALE_SIZE);
        
        if((newScale <= 2000) && (newScale >= 0) && (newScale == ((newScale / 500) * 500))){
            char str[10];
            
            UART_PutString(UART_OUT_STR_SET_YSCALE);
            sprintf(str, "%d mV\r\n", newScale);
            UART_PutString(str);
            yscale = newScale;
            
        }else UART_PutString(UART_OUT_STR_INPUT_ERR);
        
    }else{ //err
        
        UART_PutString(UART_OUT_STR_INPUT_ERR);
        
    }
    
    return;
}

void DMA_0_ISR(void){
    
    
    //once the DMA has made a complete round of 2048 data points/both descriptors
    //then move all of new data into actively drawn data at once
    //drawing handled in main
    
    if(ch1_dma_desc_position){
        
        ch1_dma_desc_position = 0;
        int i;
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
            
            ch1_currdata[i] = ch1_newdata[i];
        }
        
    }else ++ch1_dma_desc_position;
    
    //DMA completion ISR (slide 3-30)
    //Clear interrupt
    Cy_DMA_Channel_ClearInterrupt(DMA_0_HW, DMA_0_DW_CHANNEL);
}

void DMA_1_ISR(void){
    
    //once the DMA has made a complete round of 2048 data points/both descriptors
    //then move all of new data into actively drawn data at once
    //drawing handled in main
    
    if(ch2_dma_desc_position){
        
        ch2_dma_desc_position = 0;
        int i;
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
            
            ch2_currdata[i] = ch2_newdata[i];
        }
        
    }else ++ch2_dma_desc_position;
    
    //DMA completion ISR (slide 3-30)
    //Clear interrupt
    Cy_DMA_Channel_ClearInterrupt(DMA_1_HW, DMA_1_DW_CHANNEL);
}

void ShowStartupScreen(void){
    /* Set font size, foreground and background colors */
    GUI_SetFont(GUI_FONT_32B_1);
    GUI_SetBkColor(GUI_BLUE);
    GUI_SetColor(GUI_LIGHTMAGENTA);
    GUI_Clear();
  
    GUI_SetTextAlign(GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_DispStringAt("TinyScope", 160, 120);
    
    GUI_SetFont(GUI_FONT_24B_1);
    GUI_SetColor(GUI_LIGHTGREEN);
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("CSE 121/L Final Project", 160, 160);
    
    GUI_SetTextAlign(GUI_TA_HCENTER);
    GUI_DispStringAt("Lillian C. Gwendolyn", 152, 180);
}

//edited to use global constants, as there is no reason to differ this
void drawBackground(){
                
    
    GUI_SetBkColor(GUI_BLACK);
    GUI_SetColor(GUI_DARKCYAN);
    GUI_FillRect(0, 0, LCD_XSIZE, LCD_YSIZE);
    GUI_SetPenSize(1);
    GUI_SetColor(GUI_LIGHTGRAY);
    //expanded by a few pixels so that drawing the waves does not mess up the edges
    GUI_DrawRect(LCD_XMARGIN - 2, LCD_YMARGIN, LCD_XSIZE - LCD_XMARGIN + 1, LCD_YSIZE - LCD_YMARGIN);
    GUI_SetLineStyle(GUI_LS_DOT);
}

//edited to use global constants, as there is no reason to differ this
//only potentially different one is xdiv/ydiv
void drawGrid(int xdiv, int ydiv){
    
    int xstep = (LCD_XSIZE - (LCD_XMARGIN * 2)) / xdiv;
    int ystep = (LCD_YSIZE - (LCD_YMARGIN * 2)) / ydiv;

    GUI_SetPenSize(1);
    GUI_SetColor(GUI_LIGHTGRAY);
    GUI_SetLineStyle(GUI_LS_DOT);
    
    int i;
    for(i = 1; i < ydiv; i++){
        GUI_DrawLine(LCD_XMARGIN, LCD_YMARGIN + (i * ystep), LCD_XSIZE - LCD_XMARGIN, LCD_YMARGIN + (i * ystep));
    }
    
    for(i = 1; i < xdiv; i++){
        GUI_DrawLine(LCD_XMARGIN + (i * xstep), LCD_YMARGIN, LCD_XMARGIN + (i * xstep), LCD_YSIZE - LCD_YMARGIN);
    }
}

//edited to use global constants, as there is no reason to differ this
void printScaleSettings(){ 
    char str[20];

    GUI_SetBkColor(GUI_DARKCYAN); //set background color
    GUI_SetFont(GUI_FONT_16B_1); //set font
    GUI_SetColor(GUI_LIGHTGRAY); //set foreground color
    if(xscale >= 1000){
        sprintf(str, "Xscale: %0d ms/div", xscale / 1000);
    }else{
        sprintf(str, "Xscale: %0d us/div", xscale);
    }
    
    GUI_DispStringAt(str, DISPLAY_SCALE_XPOS, DISPLAY_SCALE_YPOS);
    
    sprintf(str, "Yscale: %0d.%0d V/div", yscale / 1000, (yscale % 1000) / 100);
    GUI_DispStringAt(str, DISPLAY_SCALE_XPOS, DISPLAY_SCALE_YPOS + 20);
    
}
//edited to use global constants, as there is no reason to differ this
void printFrequency(){ 
    char str[20];

    GUI_SetBkColor(GUI_DARKCYAN); //set background color
    GUI_SetFont(GUI_FONT_16B_1); //set font
    GUI_SetColor(GUI_LIGHTGRAY); //set foreground color

    sprintf(str, "Ch 1 Freq: %0d Hz", ch1_freq);
    GUI_DispStringAt(str, DISPLAY_FREQ_XPOS, DISPLAY_FREQ_YPOS);
    sprintf(str, "Ch 2 Freq: %0d Hz", ch2_freq);
    GUI_DispStringAt(str, DISPLAY_FREQ_XPOS, DISPLAY_FREQ_YPOS + 20);
}

//custom function to plot waves via drawing a line between points
void plotWave(int ystart, uint16_t ydraw[], uint32_t color){
                
    GUI_SetPenSize(2);
    GUI_SetColor(color);

    int i;
    for(i = 0; i < MAXPOINTS - 1; ++i){
        
        //LCD YSIZE - nums so that stuff higher on the screen means higher voltage instead of lower voltage
        //so that it won't look wierd if you choose a positive trigger and get a negative one
        GUI_DrawLine(i + LCD_XMARGIN, LCD_YSIZE - (ydraw[i] + ystart + LCD_YMARGIN),
                    i + LCD_XMARGIN + 1, LCD_YSIZE - (ydraw[i + 1] + ystart + LCD_YMARGIN));
    }
    return;
}

//function that scales adc data into useable plottable y values
void scaleDrawValues(){
    
    //ch1 & ch2
    //step 1 determine what our x increment is to see how many we need to plot
    //pixels per div = MAXPOINTS / 10 (x width / num x divisions (10)) = 30
    //* 4 because 4us per sample
    int sampperpix = xscale / (4 * PIX_PER_XDIV);
    //int sampperpix = 2;
    
    uint16_t sample_tracker = 0;
    uint32_t math_temp = 0;
    
    int i;
    for(i = 0; i < MAXPOINTS; ++i){
        //step 2 use formulae for finding the actual scaling of the y at given xscaling
        math_temp = ch1_currdata[(sample_tracker + ch1_trigger_start) % CHANNEL_DATA_SIZE] * MAX_VOLTAGE * PIX_PER_YDIV;
        ch1_currydraw[i] = math_temp / (ADC_MAX * yscale);
        
        math_temp = ch2_currdata[(sample_tracker + ch2_trigger_start) % CHANNEL_DATA_SIZE] * MAX_VOLTAGE * PIX_PER_YDIV;
        ch2_currydraw[i] = math_temp / (ADC_MAX * yscale);
        
        sample_tracker += sampperpix;
    }
    
    return;
}

//function that smoothes adc data for use to find trigger and freq
//actually useless, both frequency and trigger functions perform better with unsmoothed data
//poor algorythm, commented out but left in for propriety
/*
void smoothData(){
    
    int i;
    for(i = SMOOTH_DATA_START; i < CHANNEL_DATA_SIZE; ++i){
        
        //smoothing algorythm provided in project details by prof
        ch1_smoothdata[i] = (ch1_currdata[i - 4] +
                        (2 * ch1_currdata[i - 3]) +
                        (3 * ch1_currdata[i - 2]) +
                        (2 * ch1_currdata[i - 1]) +
                        ch1_currdata[i]) / 9;
        
        ch2_smoothdata[i] = (ch2_currdata[i - 4] +
                        (2 * ch2_currdata[i - 3]) +
                        (3 * ch2_currdata[i - 2]) +
                        (2 * ch2_currdata[i - 1]) +
                        ch2_currdata[i]) / 9;
    }
    
    return;
}*/

//function that finds frequencies of waves
//can use either smoothed data or unsmoothed data to find freq
//suprisingly unsmoothed seems to be more accurate
void findFrequencies(){
	
	int starting_cross = 0;
	
	//find freq of first channel
    int i;
	for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
		if((ch1_currdata[i] * MAX_VOLTAGE / ADC_MAX) <= (MAX_VOLTAGE / 2)
		    && ((i + 1) < CHANNEL_DATA_SIZE)
		    && ((ch1_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) > (MAX_VOLTAGE / 2))){
			
			starting_cross = i;
			++i;//increase value so we dont retrigger immediately
			break;
		}
	}
    
    //proceed through remaining values to find second cross, increment distance meanwhile -
    for( ; i < CHANNEL_DATA_SIZE; ++i){
        if((ch1_currdata[i] * MAX_VOLTAGE / ADC_MAX) <= (MAX_VOLTAGE / 2)
		    && ((i + 1) < CHANNEL_DATA_SIZE)
		    && ((ch1_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) > (MAX_VOLTAGE / 2))) break;
    }
    
    ch1_freq = ADC_FREQUENCY / (i - starting_cross);
	
	
	//find freq of second channel
	starting_cross = 0; //not strictly necessary but looks pretty :3
	for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
		if((ch2_currdata[i] * MAX_VOLTAGE / ADC_MAX) <= (MAX_VOLTAGE / 2)
		    && ((i + 1) < CHANNEL_DATA_SIZE)
		    && ((ch2_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) > (MAX_VOLTAGE / 2))){
			
			starting_cross = i;
			++i;//increase value so we dont retrigger immediately
			break;
		}
	}
    
    //proceed through remaining values to find second cross, increment distance meanwhile -
    for( ; i < CHANNEL_DATA_SIZE; ++i){
        if((ch2_currdata[i] * MAX_VOLTAGE / ADC_MAX) <= (MAX_VOLTAGE / 2)
		    && ((i + 1) < CHANNEL_DATA_SIZE)
		    && ((ch2_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) > (MAX_VOLTAGE / 2))) break;
    }
    
    ch2_freq = ADC_FREQUENCY / (i - starting_cross);
	
	return;
}

//function that finds starting trigger point of waves
//can use either smoothed data or unsmoothed data to find freq
//suprisingly unsmoothed seems to be more accurate
void findTriggers(){
    
    if(RunMode == freerunning){
        
        ch1_trigger_start = 0;
        ch2_trigger_start = 0;
        return;
        
    }
    
    int i;
    
    if((TrigChan == 1) && (TrigSlope == pos)){
        
        ch2_trigger_start = 0;
        
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
        
            if(((ch1_currdata[i] * MAX_VOLTAGE / ADC_MAX) < TrigLevel)
		        && ((i + 1) < CHANNEL_DATA_SIZE)
		        && ((ch1_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) >= TrigLevel)){
                
                ch1_trigger_start = i;
                return;
            }
        }
    }
    
    if((TrigChan == 1) && (TrigSlope == neg)){
        
        ch2_trigger_start = 0;
        
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
        
            if(((ch1_currdata[i] * MAX_VOLTAGE / ADC_MAX) > TrigLevel)
		        && ((i + 1) < CHANNEL_DATA_SIZE)
		        && ((ch1_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) <= TrigLevel)){
                
                ch1_trigger_start = i;
                return;
            }
        }
    }
    
    if((TrigChan == 2) && (TrigSlope == pos)){
        
        ch1_trigger_start = 0;
        
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
        
            if(((ch2_currdata[i] * MAX_VOLTAGE / ADC_MAX) < TrigLevel)
		        && ((i + 1) < CHANNEL_DATA_SIZE)
		        && ((ch2_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) >= TrigLevel)){
                
                ch2_trigger_start = i;
                return;
            }
        }
    }
    
    if((TrigChan == 2) && (TrigSlope == neg)){
        
        ch1_trigger_start = 0;
        
        for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
        
            if(((ch2_currdata[i] * MAX_VOLTAGE / ADC_MAX) > TrigLevel)
		        && ((i + 1) < CHANNEL_DATA_SIZE)
		        && ((ch2_currdata[i + 1] * MAX_VOLTAGE / ADC_MAX) <= TrigLevel)){
                
                ch2_trigger_start = i;
                return;
            }
        }
    }
    
    return;
}

int main(void){
    __enable_irq(); /* Enable global interrupts. */
    
    //background inits b4 gui stuff begins
    
    //other things to do before starting the main loop
    int i;
    for(i = 0; i < CHANNEL_DATA_SIZE; ++i){
        
        ch1_currdata[i] = 0;
        ch2_currdata[i] = 0;
        
        ch1_newdata[i] = 0;
        ch2_newdata[i] = 0;
        
    }    
    
    //UART Init (slide 4-43 / 0-43)
    cy_en_scb_uart_status_t init_status = Cy_SCB_UART_Init(UART_HW,  &UART_config, &UART_context);
    
    if(init_status != CY_SCB_UART_SUCCESS){
        //handle_error();
    }
    
    Cy_SCB_UART_Enable(UART_HW);
    UART_Start();
    
    //used following link to set up UART
    //https://www.youtube.com/watch?v=M-DjaxhYr70
    //removes buffer for getchar and putchar, eliminating 'unresponsive' feeling
    //still some response delay due to waiting in main however
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    //init channel config (slide 3-27)
    //Allocate descriptor
    cy_stc_dma_channel_config_t DMA_0_channelConfig;
    //Set parameters based on settings of DMA component
    DMA_0_channelConfig.descriptor = &DMA_0_Descriptor_1;
    //Start of descriptor chain
    DMA_0_channelConfig.preemptable = DMA_0_PREEMPTABLE;
    DMA_0_channelConfig.priority = DMA_0_PRIORITY;
    DMA_0_channelConfig.enable = false;
    DMA_0_channelConfig.bufferable = DMA_0_BUFFERABLE;
    
    //init DMA (slide 3-26 second one)
    //desc 1
    //Initialize each descriptor
    Cy_DMA_Descriptor_Init(&DMA_0_Descriptor_1, &DMA_0_Descriptor_1_config);
    //Set up source and destination addresses
    //setting up source and destination (slide 3-26 first one)
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_0_Descriptor_1, (uint32_t *) &(SAR->CHAN_RESULT[0]));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_0_Descriptor_1, ch1_newdata);
    
    //desc 2
    //Initialize each descriptor
    Cy_DMA_Descriptor_Init(&DMA_0_Descriptor_2, &DMA_0_Descriptor_2_config);
    //Set up source and destination addresses
    //setting up source and destination (slide 3-26 first one)
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_0_Descriptor_2, (uint32_t *) &(SAR->CHAN_RESULT[0]));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_0_Descriptor_2, ch1_newdata + (CHANNEL_DATA_SIZE / 2));
    
    //init channel config (slide 3-27)
    //Allocate descriptor
    cy_stc_dma_channel_config_t DMA_1_channelConfig;
    //Set parameters based on settings of DMA component
    DMA_1_channelConfig.descriptor = &DMA_1_Descriptor_1;
    //Start of descriptor chain
    DMA_1_channelConfig.preemptable = DMA_1_PREEMPTABLE;
    DMA_1_channelConfig.priority = DMA_1_PRIORITY;
    DMA_1_channelConfig.enable = false;
    DMA_1_channelConfig.bufferable = DMA_1_BUFFERABLE;
    
    //init DMA (slide 3-26 second one)
    //desc 1
    //Initialize each descriptor
    Cy_DMA_Descriptor_Init(&DMA_1_Descriptor_1, &DMA_1_Descriptor_1_config);
    //Set up source and destination addresses
    //setting up source and destination (slide 3-26 first one)
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_1_Descriptor_1, (uint32_t *) &(SAR->CHAN_RESULT[1]));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_1_Descriptor_1, ch2_newdata);
    
    //desc 2
    //Initialize each descriptor
    Cy_DMA_Descriptor_Init(&DMA_1_Descriptor_2, &DMA_1_Descriptor_2_config);
    //Set up source and destination addresses
    //setting up source and destination (slide 3-26 first one)
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_1_Descriptor_2, (uint32_t *) &(SAR->CHAN_RESULT[1]));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_1_Descriptor_2, ch2_newdata + (CHANNEL_DATA_SIZE / 2));
    
    //init DMA (slide 3-26 second one)
    /* Initialize channel */
    Cy_DMA_Channel_Init(DMA_0_HW, DMA_0_DW_CHANNEL, &DMA_0_channelConfig); //channel config is structure containing channel init params
    /* Enable channel */
    Cy_DMA_Channel_Enable(DMA_0_HW, DMA_0_DW_CHANNEL);
    Cy_DMA_Enable(DMA_0_HW);
    
    //init DMA (slide 3-26 second one)
    /* Initialize channel */
    Cy_DMA_Channel_Init(DMA_1_HW, DMA_1_DW_CHANNEL, &DMA_1_channelConfig); //channel config is structure containing channel init params
    /* Enable channel */
    Cy_DMA_Channel_Enable(DMA_1_HW, DMA_1_DW_CHANNEL);
    Cy_DMA_Enable(DMA_1_HW);
    
    //init DMA interrupts (slide 3-29)
    //Initialize and enable the interrupt from DMA
    Cy_SysInt_Init(&DMA_0_INT_cfg, DMA_0_ISR);
    NVIC_EnableIRQ(DMA_0_INT_cfg.intrSrc);
    //Enable DMA interrupt source
    Cy_DMA_Channel_SetInterruptMask(DMA_0_HW, DMA_0_DW_CHANNEL, CY_DMA_INTR_MASK);
    //Enable DMA after all setup is completed
    
    //init DMA interrupts (slide 3-29)
    //Initialize and enable the interrupt from DMA
    Cy_SysInt_Init(&DMA_1_INT_cfg, DMA_1_ISR);
    NVIC_EnableIRQ(DMA_1_INT_cfg.intrSrc);
    //Enable DMA interrupt source
    Cy_DMA_Channel_SetInterruptMask(DMA_1_HW, DMA_1_DW_CHANNEL, CY_DMA_INTR_MASK);
    //Enable DMA after all setup is completed

    /* Initialize EmWin Graphics */
    GUI_Init();
   
    /* Display the startup screen for 5 seconds */
    ShowStartupScreen();
    
    //init ADC
    ADC_Start();
    ADC_StartConvert();
    
    for(;;)
    {
		//delay first so the start skip works better
		Cy_SysLib_Delay(200);
		
        //poll rx fifo empty
        //if rx fifo status = not empty then call terminal function
        if(UART_GetRxFifoStatus() & CY_SCB_UART_RX_NOT_EMPTY) manageUART();
        
        //if not running then only check for UART communication stuff
		if(!isRunning) continue;
		
		//if just started running then draw background
		if(!hasPrintedBackground){
			drawBackground();
			hasPrintedBackground = true;
		}
        //do not otherwise constantly redraw background, makes screen 'blink'
        
        //cycle in potentiometer values
        //pot values used for y start for drawing waves
        //SAR result is potentiometer output
        ch1_prevpot = ch1_currpot;
        ch1_currpot = SAR->CHAN_RESULT[2] * MAYPOINTS / ADC_MAX;
        
        ch2_prevpot = ch2_currpot;
        ch2_currpot = SAR->CHAN_RESULT[3] * MAYPOINTS / ADC_MAX;
    
        //save old values to null out the previous waves without redrawing entire background
        for(i = 0; i < MAXPOINTS; ++i){
        
            ch1_prevydraw[i] = ch1_currydraw[i];
            ch2_prevydraw[i] = ch2_currydraw[i];
        
        }
        
        //redraw old waves with background color
        plotWave(ch1_prevpot, ch1_prevydraw, GUI_DARKCYAN);
        plotWave(ch2_prevpot, ch2_prevydraw, GUI_DARKCYAN);
        
        //get new values
        //smoothData(); //smoothing results in worse data somehow, left in for propriety
        findTriggers();
        scaleDrawValues();
        
        //draw new waves
        plotWave(ch1_currpot, ch1_currydraw, GUI_YELLOW);
        plotWave(ch2_currpot, ch2_currydraw, GUI_GREEN);
        
        //redraw info on display
		findFrequencies();
        drawGrid(GRID_XDIVISIONS, GRID_YDIVISIONS); //redraw grid
        printScaleSettings(); //redraw scale settings
        printFrequency(); //redraw frequency settings
    }
}
