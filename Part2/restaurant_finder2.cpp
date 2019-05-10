/*Authors: Veronica Salm and Logan McDonald
CMPUT 274 LBL EA2
Fall 2016

Restaurant Finder Version 2: This program has two modes. In mode 0, the user can use the
joystick to move a cursor around a scrollable map of Edmonton. When the joystick button
is pressed, the program switches to mode 1, in which a list of 20 nearby restaurants is
displayed on the screen. The restaurants are displayed in order of nearest to farthest
fron the cursor position. When the joystick button is pressed again to select a restaurant,
the program returns to mode 1, displaying the cursor and selected restaurant in the center
of the screen.

In this second version, the user can filter the restaurants listed by rating and scroll
through more than 20 restaurants.
*/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <SD.h>

#include "lcd_image.h"

// standard U of A library settings, assuming Atmel Mega SPI pins
#define SD_CS    5  // Chip select line for SD card
#define TFT_CS   6  // Chip select line for TFT display
#define TFT_DC   7  // Data/command line for TFT
#define TFT_RST  8  // Reset line for TFT (or connect to +5V)

#define JOY_SEL 9
#define JOY_VERT_ANALOG 0
#define JOY_HORIZ_ANALOG 1

//define LED pins and potentiometer analog pin for ratings
int ledPin[5] = {2, 3, 4, 10, 11};
#define P_ANALOG 2

#define JOY_DEADZONE 64 //the deadzone of the joystick - can be very small since the joystick is calibrated
#define JOY_STEPS_PER_PIXEL 128

#define SCREEN_SIZE_X 128 //horizontal size of screen
#define SCREEN_SIZE_Y 160 //vertical size of screen
#define CURSOR_SIZE 3 //the size of the cursor (a 3x3 pixel square)

#define MILLIS_PER_FRAME 30
#define RUNTIME_CONSTANT 100 // increasing this makes the scroll speed slower when restaurants are displayed

int RUNTIME;// specifies how fast the highlighted restaurant will change when the restaurants are displayed
			// depends upon the RUNTIME_CONSTANT and changes

// the maximum permissible map coordinates are one screen size less than the image size
#define MAX_IMAGE_X 1920 //2048-SCREEN_SIZE_X
#define MAX_IMAGE_Y 1888 //2048-SCREEN_SIZE_Y

#define REST_START_BLOCK 4000000 // The location of the beginning of raw written data (first block of restaurants) on the SD card
#define NUM_REST 1066 // The total number of restaurants

#define MAX_LIST_IDX 19 //used in determining the list of restaurants to print in MODE1

Sd2Card card;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST); //tft library

int g_cursorX = (SCREEN_SIZE_X/2)-(CURSOR_SIZE/2); // cursor pixel position - puts the 3x3 pixel cursor roughly in the center of the screen
int g_cursorY = (SCREEN_SIZE_Y/2)-(CURSOR_SIZE/2);
int g_prevX = g_cursorX; // previously drawn position of the cursor, which starts the same as the initial cursor position
int g_prevY = g_cursorY;

int JOY_V_CENTRE; //the central (resting) location of the joystick, both vertical and horizontal
int JOY_H_CENTRE;

//map coordinates - start at center of map (half of the image size - half of the screen size)
int IMAGE_X = 2048/2-SCREEN_SIZE_X/2; //x coordinate of the image
int IMAGE_Y = 2048/2-SCREEN_SIZE_Y/2; //image y coordinate

bool MODE1 = false; //the program mode is 0 if false and 1 if true
bool isPressed = false;
bool update = false;

int rating = 0; //the restaurant rating, from 0 to 5
int prev_rating = rating;
// unsigned int lastRest; //the index of the last restaurant in the RestDist struct

/*Initializes the map image and specifies the width and height.
* Parameters:
* filename
* width of image (in pixels)
* height of image (in pixels)
*/
lcd_image_t map_image = { "yeg-big.lcd", 2048, 2048 };

/* Draws the referenced image to the LCD screen.
* Parameters:
* img           : the image to draw
* tft           : the initialized tft struct
* icol, irow    : the upper-left corner of the image patch to draw
* scol, srow    : the upper-left corner of the screen to draw to
* width, height : controls the size of the patch drawn.
*/
void lcd_image_draw(lcd_image_t *img, Adafruit_ST7735 *tft,
	uint16_t icol, uint16_t irow,
	uint16_t scol, uint16_t srow,
	uint16_t width, uint16_t height) {
	File file;
	// Open requested file on SD card if not already open
	if ((file = SD.open(img->file_name)) == NULL) {
		Serial.print("File not found:'");
		Serial.print(img->file_name);
		Serial.println('\'');
		return;
	}

	// Setup display to receive window of pixels
	tft->setAddrWindow(scol, srow, scol+width-1, srow+height-1);

	for (uint16_t row=0; row < height; row++) {
		uint16_t pixels[width];

		// Seek to start of pixels to read from, need 32 bit arith for big images
		uint32_t pos = ( (uint32_t) irow +  (uint32_t) row) *
		(2 *  (uint32_t) img->ncols) +  (uint32_t) icol * 2;
		file.seek(pos);

		// Read row of pixels
		if (file.read((uint8_t *) pixels, 2 * width) != 2 * width) {
			file.close();
			return;
		}

		// Send pixels to display
		for (uint16_t col=0; col < width; col++) {
			uint16_t pixel = pixels[col];

			// pixel bytes in reverse order on card
			pixel = (pixel << 8) | (pixel >> 8);
			tft->pushColor(pixel);
		}
	}
	file.close();
}

/*Performs necessary setup functions: Initializes TFT, the joystick,
and the SD card. Also prints the startup message on the LCD screen
and initializes the LED pins for the rating system. */
void setup(void) {
	// initialize the SD card
	Serial.print("Initializing SD card...");
	if (!SD.begin(SD_CS)) {
		Serial.println("failed!");
		while (true) {} // something is wrong
	}
	else {
		Serial.println("OK!");
	}

	// Init TFT
	tft.initR(INITR_BLACKTAB);

	// Init joystick
	pinMode(JOY_SEL, INPUT);
	digitalWrite(JOY_SEL, HIGH); // enables pull-up resistor - required!
	Serial.println("Joystick initialized!");

	// initialize SPI (serial peripheral interface)
	// communication between the Arduino and the SD controller
	Serial.print("Initializing SPI communication for raw reads...");
	if (!card.init(SPI_HALF_SPEED, SD_CS)) {
		Serial.println("failed!");
		while (true) {}
	}
	else {
		Serial.println("OK!");
	}

	//read the horizontal and vertical resting states of the joystick
	JOY_V_CENTRE = analogRead(JOY_VERT_ANALOG);
	JOY_H_CENTRE = analogRead(JOY_HORIZ_ANALOG);

	//Display instructions when program loads
	bool sel = digitalRead(JOY_SEL);
	tft.fillScreen(0);
	tft.setCursor(0,0);
	tft.setTextColor(ST7735_WHITE);
	tft.setTextWrap(true);
	tft.setTextSize(1);
	tft.println("Restaurant finder: ");
	tft.println("1.Move the cursor");
	tft.println("2.Change rating to");
	tft.println(" filter restaurants");tft.println();
	tft.println("3.Press the button"); tft.println();
	tft.println(" Restaurants are");
	tft.println(" shown by increasing");
	tft.println(" distance. Scroll");
	tft.println(" to see more.");
	tft.println();
	tft.println("3.Press button again  to select restaurant");
	tft.println(" Cursor highlights");
	tft.println(" restaurant location");
	tft.setCursor(0, 152);
	//The user must press and release the button to start the program
	tft.println("Press button to start");

	// wait for the button to be pressed and then released
	while(sel) {
		sel = digitalRead(JOY_SEL);
	}
	while(!sel) {
		sel = digitalRead(JOY_SEL);
	}

	//finally, set all LED pins to output
	for (int i = 0; i < 5; ++i) {
		pinMode(ledPin[i], OUTPUT);
	}
}

/*Draws the initial map image and cursor*/
void initializeMap() {
	//draw the initial map image
	lcd_image_draw(&map_image, &tft, IMAGE_X, IMAGE_Y, 0, 0, SCREEN_SIZE_X, SCREEN_SIZE_Y);

	// draw cursor in the center of the map
	tft.fillRect(g_cursorX, g_cursorY, CURSOR_SIZE, CURSOR_SIZE, ST7735_RED);
}

/*Switches between program modes when the joystick button is pressed*/
void modetoggle() {
	bool sel = digitalRead(JOY_SEL);

	//when the button is pushed down
	if (!sel && !isPressed) {
		//if the mode is 0, change the mode to 1
		if (!MODE1) {
			MODE1 = true;
		}
		//if the mode is 1, change the mode to 0
		else {
			MODE1 = false;
		}
		//the button is now pressed until it is released (until sel is true again)
		isPressed = true;
	}
	//when the button is released, isPressed becomes false
	if (sel) {
		isPressed = false;
	}
}

/*In mode 0, scans the joystick for any user input and updates the cursor position to match.*/
void scanJoystick0() {
	int v = analogRead(JOY_VERT_ANALOG);
	int h = analogRead(JOY_HORIZ_ANALOG);

	// update g_cursorX & g_cursorY, the x and y coordinates of the cursor on the screen
	if (abs(v - JOY_V_CENTRE) > JOY_DEADZONE) { //if vertical joystick is outside of deadzone
		int delta = (v - JOY_V_CENTRE) / JOY_STEPS_PER_PIXEL;
		g_cursorY = constrain(g_cursorY + delta, 0, SCREEN_SIZE_Y-CURSOR_SIZE);
		//using SCREEN_SIZE-CURSOR_SIZE allows the entire cursor to remain visible when it hits the bottom of the map
	}

	if (abs(h - JOY_H_CENTRE) > JOY_DEADZONE) { //if horizontal is outside of deadzone
		int delta = (h - JOY_H_CENTRE) / JOY_STEPS_PER_PIXEL;
		g_cursorX = constrain(g_cursorX + delta, 0, SCREEN_SIZE_X-CURSOR_SIZE); //cursor remains visible at edge of map
	}
}

/*
Draws the map image in the location of the old cursor before redrawing the
cursor in its new location.
The map is updated if the cursor has reached the edge of the screen (the top,
left, right, or bottom), but only if it has not reached the edge of the image.
*/
void updateDisplay() {
	//the center of the window (x and y coordinates of the map)
	int centerX = (SCREEN_SIZE_X/2) - (CURSOR_SIZE/2);
	int centerY = (SCREEN_SIZE_Y/2) - (CURSOR_SIZE/2);

	//change in x and y - determines when the cursor has hit the edge of the screen
	int changeX = (g_cursorX) - centerX;
	int changeY = (g_cursorY) - centerY;

	/*Redraws the map image starting at the previous x and y coordinates, but only redraws a square the size of the cursor.*/
	lcd_image_draw(&map_image, &tft, g_prevX+IMAGE_X, g_prevY+IMAGE_Y, g_prevX, g_prevY, CURSOR_SIZE, CURSOR_SIZE);

	//redraw the cursor in the new position
	tft.fillRect(g_cursorX, g_cursorY, CURSOR_SIZE, CURSOR_SIZE, ST7735_RED);

	g_prevX = g_cursorX; //the previous cursor becomes the current cursor
	g_prevY = g_cursorY;

	//if the cursor has hit any edge of the screen
	if ((changeX) > (SCREEN_SIZE_X/2-CURSOR_SIZE) || changeX <= (0-(SCREEN_SIZE_X/2-CURSOR_SIZE/2)) || (changeY) > (SCREEN_SIZE_Y/2-CURSOR_SIZE)|| changeY <= (0-(SCREEN_SIZE_Y/2-CURSOR_SIZE/2))) {
		//do not refresh the screen if the cursor is at any of the map edges
		if (g_cursorX+IMAGE_X == 0 || (g_cursorX+IMAGE_X) >= (2048-CURSOR_SIZE) || g_cursorY+IMAGE_Y == 0 || g_cursorY+IMAGE_Y == 2048-CURSOR_SIZE) {}
		//in all other situations where the cursor is at the edge of the screen, update the display
		else {
			//update the map coordinates
			IMAGE_X = constrain(IMAGE_X + changeX, 0, MAX_IMAGE_X);
			IMAGE_Y = constrain(IMAGE_Y + changeY, 0, MAX_IMAGE_Y);
			//put the cursor in the center of the screen
			g_cursorX = SCREEN_SIZE_X/2;
			g_cursorY = SCREEN_SIZE_Y/2;
			//redraw the map
			lcd_image_draw(&map_image, &tft, IMAGE_X, IMAGE_Y, 0, 0, SCREEN_SIZE_X, SCREEN_SIZE_Y);
		}
	}
}

/*Reads the potentiometer voltage and converts it to a
  rating from 0 to 5. */
void voltage_to_rating() {
  int voltage = analogRead(P_ANALOG);

  // The rating is determined as follows, so that each bin is even:
  // 0 when voltage is between 0 and 171
  // 1 when voltage is between 172 and 342
  // 2 when voltage is between 343 and 513
  // 3 when voltage is between 514 and 684
  // 4 when voltage is between 685 and 855
  // 5 when voltage is between 856 and 1023
  rating = voltage/171;
}

/*Uses the current rating and updates the LEDs to match.*/
int updateLEDs() {
    for (int i = 0; i < rating; ++i) {
      digitalWrite(ledPin[i], HIGH);
    }
    for (int i = rating; i < 5; i++) {
      digitalWrite(ledPin[i], LOW);
    }
}

/*
Checks the current voltage from the potentiometer, updates the rating accordingly
and calls the updateLEDs() function.
*/
void checkRating(int* voltage, int* new_voltage) {
	//check if rating has changed
	*new_voltage = analogRead(P_ANALOG)/10;
	prev_rating = rating;
	//only convert voltage to rating and update LEDs if the voltage has changed
	if (*new_voltage != *voltage) {
			voltage_to_rating();
			*voltage = *new_voltage;
	}
	// update the LEDs to mirror the change in rating
	updateLEDs();
}

/*Defines a restaurant struct with the following parameters:
latitude, longitude, rating (0-10), and name */
struct restaurant {
	int32_t lat;
	int32_t lon;
	uint8_t rating;
	char name[55];
};

/*Defines a restaurant in a smaller amount of memory (4 bytes) so that all
1066 restaurants will fit in the Arduino's memory.
Parameters: index (#), Manhatten distance.*/
struct RestDist {
	uint16_t index;     // index of restaurant from 0 to NUM_RESTAURANTS-1
	uint16_t dist;      // Manhatten distance to cursor position
};

// Variable declarations for the restaurants and restaurant names.
RestDist rest_dist[NUM_REST];

uint32_t prevblock = 0;
restaurant buffer[8]; // Stores one block of restaurants
restaurant r;

int previous_rest; // stores the value of the previous restaurant and the
int selected_rest; // current restaurant in the list

/*Gets restaurants from the SD card quickly by only reading from the SD
card when a new block of restaurants has been reached.*/
void get_restaurant_fast(restaurant* ptr, int i) {
	uint32_t block = REST_START_BLOCK + i/8;
	// read from the SD card only if the new block is different from previous
	if (block != prevblock ) {
		// fill the restaurant buffer with the block
		while (!card.readBlock(block, (uint8_t*) buffer)) {
			Serial.println("readBlock failed, trying again!");
		}
	}
	//the old block becomes the new one
	prevblock = block;
	*ptr = buffer[i%8];
}

// These constants are for the 2048 by 2048 map.
const int16_t map_width = 2048;
const int16_t map_height = 2048;
const int32_t lat_north = 5361858;
const int32_t lat_south = 5340953;
int32_t lon_west = -11368652;
int32_t lon_east = -11333496;

// These functions convert from lon and lat to x and y coordinates on the map
int16_t lon_to_x(int32_t lon) {
	return map(lon, lon_west, lon_east, 0, map_width);
}

int16_t lat_to_y(int32_t lat) {
	return map(lat, lat_north, lat_south, 0, map_height);
}



/* Gets the restaurant distances from the current cursor location and stores them in rest_dist[] */
void get_rest_dist(RestDist* rest_dist, int i, int cursor_coordinate_x, int cursor_coordinate_y, int* j) {
	get_restaurant_fast(&r, i);
	int rest_x = lon_to_x(r.lon);
	int rest_y = lat_to_y(r.lat);

	/*if the restaurant has a rating equal to or greater than the specified rating, store it
	and increment j.
	Otherwise, skip it and don't store it.
	i - the index of the restaurants in the "restaurant" struct array
	j - the index of the restaurants in the "RestDist" struct array*/
	if (floor((r.rating+1)/2) >= rating) {
		rest_dist[*j].index = i;
		rest_dist[*j].dist = (abs(rest_x - cursor_coordinate_x) + abs(rest_y - cursor_coordinate_y));
		++*j;
	}

}

// Swap two restaurants of RestDist struct. Used to sort the restaurants by distance
void swap_rest(RestDist *ptr_rest1, RestDist *ptr_rest2) {
	RestDist tmp = *ptr_rest1;
	*ptr_rest1 = *ptr_rest2;
	*ptr_rest2 = tmp;
}

/*Chooses the pivot used in the partition function*/
int pick_pivot(int len) {
  return len/2;
}

/*
  Rearrange the array, in place, so that
  all items left of the pivot have value at most
  the pivot and all items to the right of the pivot
  have greater than the pivot.

  Return the new index of the pivot once this is done.
*/
int partition(RestDist* array, int len, int pivot_idx) {
  swap_rest(&array[len-1], &array[pivot_idx]);
  int low = 0;
  int high = len - 2;
  while (low != high) {
    if (array[low].dist <= array[len-1].dist) {
      ++low;
    }
    else if (array[high].dist > array[len-1].dist ) {
      --high;
    }
    else {
      swap_rest(&array[high], &array[low]);
      ++low;
    }
  }
  if (array[len-1].dist > array[low].dist) {
      ++low;
  }
  swap_rest(&array[len-1], &array[low]);
  return low;
}

/*Quickly and recursively sorts the restaurants by
	partitioning low and high values around pivots
	and then recombining sorted arrays. */
void qsort(RestDist* array, int len) {
  if (len <= 1) return; // sorted already

  // choose the pivot
  int pivot_idx = pick_pivot(len);

  // partition around the pivot and get the new
  // pivot position
  pivot_idx = partition(array, len, pivot_idx);

  // recurse on the halves before and after the pivot
  qsort(array, pivot_idx);
  qsort(array + pivot_idx + 1, len - pivot_idx - 1);
}

/*Updates which name is highlighted as restaurants are scrolled
	through in Mode 1.*/
void updatenames(int list) {
	//the previous restaurant is printed in white on a black background
	tft.setCursor(0, previous_rest*8);
	tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
	get_restaurant_fast(&r, rest_dist[list + previous_rest].index);
	tft.print(r.name);

	//the newly selected restaurant is highlighted
	//(printed in black on a white background)
	tft.setCursor(0, selected_rest*8);
	tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
	get_restaurant_fast(&r, rest_dist[list + selected_rest].index);
	tft.print(r.name);
}

void sortRest(unsigned int* lastRest) {
	// stores all of the data for restaurant distances from the current cursor position
	int cursor_coordinate_x = IMAGE_X + g_cursorX + (CURSOR_SIZE/2);
	int cursor_coordinate_y = IMAGE_Y + g_cursorY + (CURSOR_SIZE/2);

	//get all restaurants from the SD card
	int j = 0;
	for (int i = 0; i < NUM_REST; ++i) {
		get_rest_dist(rest_dist, i, cursor_coordinate_x, cursor_coordinate_y, &j);
	}
	// establishes the index of the last restaurant stored in the rest_dist array
	*lastRest = j - 1;
	// Serial.print("lastRest: "); Serial.println(*lastRest);

	// Sort the restaurants by distance: swap them so the
	// closest one is at the beginning.
	qsort(rest_dist, *lastRest);
}

/*Displays the restaurants on the LCD screen in order of increasing
distance from the cursor's position on the map.
Based upon a list value which represents a section of 20 restaurants to display*/
void displayRest(int list) {
	tft.fillScreen(0);
	tft.setCursor(0, 0); // where the characters will be displayed
	tft.setTextWrap(false);

	// the selected list
	// Serial.print("List value: ");
	// Serial.println(list);

	//Print all restaurants on the screen, with the selected restaurant highlighted.
	for (int i = (list); i < (list+20); ++i) {
		get_restaurant_fast(&r, rest_dist[i].index);
		if (i != list + selected_rest) { // not highlighted
			tft.setTextColor(0xFFFF, 0x0000); // white characters on black background
		} else { // highlighted
			tft.setTextColor(0x0000, 0xFFFF); // black characters on white background
		}
		// print each restaurant: distance name index rating
		// Serial.print(rest_dist[i].dist); Serial.print(" "); Serial.print(r.name); Serial.print("  "); Serial.print(i);
		// Serial.print("  "); Serial.println(r.rating);
		//print the name on the LCD display
		tft.print(r.name);
		tft.print("\n");
	}
	tft.print("\n");
	Serial.println();
}

/*
The scan joystick function for mode 1
-updates the selected restaurant in the list based upon joystick position
-only accounts for vertical changes
-allows for the list to scroll when the highlight moves past the bottom or top,
except on edge cases.
*/
void scanJoystick1(int* list, unsigned int lastRest) {
	int v = analogRead(JOY_VERT_ANALOG);
	int delta = v - JOY_V_CENTRE;

	// the running time of the scroll:
	// ~600 ms when joystick is centered
	// increases proportionally to ~100 ms as the joystick is tilted
	RUNTIME = (JOY_V_CENTRE + RUNTIME_CONSTANT - (abs(JOY_V_CENTRE - v)));

	previous_rest = selected_rest;

	//if the joystick is outside the deadzone
	if (abs(delta) > JOY_DEADZONE) {
		//the selected restaurant has changed, so the names must be updated
		update = true;

		// if the list is at the very beginning and the joystick is moved up,
		// or the list is at the very end and joystick is moved down, do nothing
		if ((*list + selected_rest == lastRest && delta > 0) || (selected_rest == 0 && *list == 0 && delta < 0)) {
			update = false;
		}
		else if (delta > 0) { //if delta is positive, the joystick was moved down
			selected_rest = constrain(selected_rest + 1, -1, 20);
			// if the joystick goes past the bottom
			if (selected_rest == 20) {
				// if the next list isn't long enough to fill the screen (the number of remaining restaurants is less than 20)
				if (*list + MAX_LIST_IDX > lastRest - MAX_LIST_IDX) {
					//change the list pointer so that it is 19 restaurants from the end, so the screen is always filled
					*list = lastRest - MAX_LIST_IDX;
					//and move the pointer to the next selected restaurant
					selected_rest = MAX_LIST_IDX - (lastRest%20);
				}
				else { // draw the new list
					(*list) += 20;
					//highlight the first restaurant on the list
					selected_rest = 0;
				}
				displayRest(*list);
			}
		}
		else { //if the joystick is moved up
			selected_rest = selected_rest - 1;
			// if the joystick goes past the top of the screen
			if (selected_rest == -1) {
				// if the list is last possible one, draw the list from a multiple of 20
				if ((*list % 20) != 0) {
					*list = *list - ((lastRest + 1) % 20);
					selected_rest = lastRest % 20;
				}
				// otherwise, move the list up by 20
				else {
					*list -= 20;
					selected_rest = 19;
				}
				// highlight the last restaurant on the list
				displayRest(*list);
			}
		}
	}
}

/*
Sorts and displays the restaurants again to mirror the change in rating
Always starts the display from the closest restaurant
*/
void SortAndDisplay(int* list, unsigned int* lastRest) {
	*list = 0;
	sortRest(lastRest);
	selected_rest = 0;
	displayRest(*list);
}

/*Displays the selected restaurant and cursor in the center of the LCD screen.
The map image and cursor coordinates must be adjusted accordingly before the
screen is redrawn. The name of the restaurant is also printed at the top of the
screen.

The restaurant name is erased when the joystick is moved.*/
void locationDisplay(int list) {
	get_restaurant_fast(&r, rest_dist[list + selected_rest].index);

	//Convert the longitude and latitude of the restaurant to x and y map coordinates
	int16_t x_coordinate = lon_to_x(r.lon);
	int16_t y_coordinate = lat_to_y(r.lat);

	//update the image coordinates to match
	IMAGE_X = constrain(x_coordinate - (SCREEN_SIZE_X/2), 0, MAX_IMAGE_X);
	IMAGE_Y = constrain(y_coordinate - (SCREEN_SIZE_Y/2), 0, MAX_IMAGE_Y);

	// conditions for if the restaurant is located on the outskirts of the map
	if (x_coordinate > (MAX_IMAGE_X + (SCREEN_SIZE_X/2)) || x_coordinate < (SCREEN_SIZE_X/2)) {
		g_cursorX = x_coordinate - IMAGE_X - (CURSOR_SIZE/2);
	}
	else{
		//in all other cases, put the cursor in the center of the screen
		g_cursorX = (SCREEN_SIZE_X/2) - (CURSOR_SIZE/2);
	}
	//if the restaurant is located on or outside the top or bottom of the map
	if (y_coordinate > (MAX_IMAGE_Y + (SCREEN_SIZE_Y/2)) || y_coordinate < (SCREEN_SIZE_Y/2)) {
		g_cursorY = y_coordinate - IMAGE_Y - (CURSOR_SIZE/2);
	}
	//if within bounds
	else {
		//put the cursor in the center of the screen
		g_cursorY = (SCREEN_SIZE_Y/2) - (CURSOR_SIZE/2);
	}

	// redraw the map and cursor
	lcd_image_draw(&map_image, &tft, IMAGE_X, IMAGE_Y, 0, 0, SCREEN_SIZE_X, SCREEN_SIZE_Y);
	tft.fillRect(g_cursorX, g_cursorY, CURSOR_SIZE, CURSOR_SIZE, ST7735_RED);

	// print the name of the restaurant across the top of the screen
	tft.setCursor(0,0);
	tft.setTextWrap(true);
	tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
	tft.print(r.name);

	//wait until the joystick is moved again
	int v = analogRead(JOY_VERT_ANALOG);
	int h = analogRead(JOY_HORIZ_ANALOG);
	while (abs((v - JOY_V_CENTRE)) < JOY_DEADZONE && abs((h - JOY_H_CENTRE)) < JOY_DEADZONE) {
		v = analogRead(JOY_VERT_ANALOG);
		h = analogRead(JOY_HORIZ_ANALOG);
	}
	//Erase the restaurant name (draw over it) and the cursor.
	lcd_image_draw(&map_image, &tft, IMAGE_X, IMAGE_Y, 0, 0, SCREEN_SIZE_X, 16); //redraw in two lines in case the name was long enough to wrap
	lcd_image_draw(&map_image, &tft, g_cursorX+IMAGE_X, g_cursorY+IMAGE_Y, g_cursorX, g_cursorY, CURSOR_SIZE, CURSOR_SIZE);
}

int main() {
	init();
	Serial.begin(9600);

	setup();

	// variables to allow control of the cursor movement speed
	unsigned long long startTime = millis();
	unsigned long long curTime;

	unsigned int lastRest;

	//Draw the initial map and cursor
	initializeMap();

	//used to ensure the rating is only updated if the voltage has changed
	int voltage = analogRead(P_ANALOG)/10; //divided by 10 to account for natural fluctuations in the volage
	int new_voltage = voltage;
	int list;	// variable indicating which list of restaurants to display
		 			// in increments of 20


	//read the initial potentiometer state and convert the voltage to a rating
	voltage_to_rating();
	//update LEDs to match
	updateLEDs();

	while (true) {
		while (MODE1 == false) {
			/*Continually scan the joystick and update the display if the cursor
			location has changed*/
			scanJoystick0();
			//Check if a mode switch is required
			modetoggle();
			// only update the display if the cursor position has changed:
			if ((g_cursorX != g_prevX) || (g_cursorY != g_prevY)) {
				updateDisplay();
			}
			/* Create a variable delay, so that each loop iteration
			takes MILLIS_PER_FRAME milliseconds. This slows down the cursor
			and prevents it from moving too quickly. */
			curTime = millis();
			if (curTime - startTime < MILLIS_PER_FRAME) {
				delay(MILLIS_PER_FRAME - (curTime-startTime));
			}
			startTime = millis();

			// check to see if the rating has changed and update the LEDs
			checkRating(&voltage, &new_voltage);
		}

		// If the program mode changes to 1, display the closest restaurants to that location
		if (MODE1 == true) {
			tft.setCursor(0,0);
			tft.setTextColor(ST7735_BLACK);
			//prints a message to the user to let them know which restaurants are being printed
			tft.print(rating);
			tft.print(" star or greater:");
			// Sort and display the restaurants based upon current rating.
			SortAndDisplay(&list, &lastRest);
		}

		/*
		While the mode remains at 1, scan the joystick and update the highlighted
		restaurant using function updatenames().
		Also, make sure the highlight doesn't change too fast using the variable
		RUNTIME.
		*/
		while (MODE1 == true) {
			startTime = millis();
			scanJoystick1(&list, lastRest);
			//Check if a mode switch is required
			modetoggle();
			// change the highlighted name if the joystick has been moved up or down
			if (update == true) {
				updatenames(list);
				curTime = millis();
				if (curTime - startTime < RUNTIME) {
					delay(RUNTIME - (curTime-startTime));
				}
				update = false;
			}

			checkRating(&voltage, &new_voltage);
			// if the rating changes while the restaurants are being displayed,
			// redisplay them according to the new rating.
			if (rating != prev_rating) {
				tft.fillScreen(0);
				tft.setCursor(0,0);
				tft.setTextColor(ST7735_WHITE);
				tft.setTextWrap(true);
				//prints a message to the user to let them know which restaurants are being printed
				tft.print(rating);
				tft.print(" star or greater:");
				SortAndDisplay(&list, &lastRest);
			}
			// if the button is clicked again, place the cursor at the restaurant location
			if (MODE1 == false) {
				locationDisplay(list);
			}
		}
	}
	Serial.end();
	return 0;
}
