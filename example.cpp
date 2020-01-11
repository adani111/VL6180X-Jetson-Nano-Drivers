#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <VL6180X.h>

VL6180X sensor;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main() {
    // VL6180X *sensor = new VL6180X();
    // Open the sensor
    if (!sensor.openVL6180X()) {
        // Trouble
        printf("Unable to open VL6180X") ;
        exit(-1) ;
    }
    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(500);
    
    // 27 is the ESC key
    printf("Example for VL6180X\n");
    printf("Place object in front of sensor for reading ...\n") ;
    printf("Press the ESC key to stop the program\n");
    while(getkey() != 27){
        int distance = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) { printf("Sensor timeout!\n"); } 
	else { printf("\nDistance: %d mm ",distance); }
    }
    printf("\n\n");
    //printf("Sensor deactivated\n");
    //sensor.closeVL53L0X();
}
