#include <sh1106.h>
#include <string.h>

// Screen dimensions
uint16_t scr_width  = SCR_W;
uint16_t scr_height = SCR_H;

// Pixel drawing mode
// Whereas in most drawing operations pixels are set, use global variable to select drawing mode
// instead of passing set/reset/invert mode in each call of drawing functions
uint8_t LCD_PixelMode = LCD_PSET;

// Display image orientation
static uint8_t scr_orientation = LCD_ORIENT_NORMAL;

// Video RAM buffer
static uint8_t vRAM[(SCR_W * SCR_H) >> 3] __attribute__((aligned(4)));

// Vertical line drawing look up table for first byte
static const uint8_t LUT_FB[] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };

// Vertical line drawing look up table for last byte
static const uint8_t LUT_LB[] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };


// Send single byte command to display
// input:
//   cmd - display command
static void SH1106_cmd(uint8_t cmd) {
	// Deassert DC pin -> command transmit
	SH1106_DC_L();

	// Send command to display
	uint8_t command = cmd;
	HAL_SPI_Transmit ( &SH1106_SPI_PORT,  &command, 1, 20) ;
}

// Send double byte command to display
// input:
//   cmd1 - first byte of double-byte command
//   cmd2 - second byte of double-byte command
static void SH1106_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	// Deassert DC pin -> command transmit
	SH1106_DC_L();

	// Send double byte command to display
	uint8_t command[2];
	command[0] = cmd1;
	command[1] = cmd2;
	HAL_SPI_Transmit ( &SH1106_SPI_PORT,  command, 2, 20) ;

}

/*
// Send data byte to display
// input:
//   data - data byte
static void SH1106_data(uint8_t data) {
	// Assert DC pin -> data transmit
	SH1106_DC_H();

	// Send data byte to display
	SPIx_Send(&SH1106_SPI_PORT,data);
}
*/

// Initialize the display control GPIO pins
/*
void SH1106_InitGPIO(void) {
	// Enable the GPIO peripheral(s) clock
	RCC->AHBENR |= SH1106_GPIO_PERIPH;

	// Configure CS pin as push-pull output with pull-up
	GPIO_set_mode(SH1106_CS_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,SH1106_CS_PIN);
	GPIO_out_cfg(SH1106_CS_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,SH1106_CS_PIN);
	SH1106_CS_H();

	// Configure DC pin as push-pull output with pull-up
	GPIO_set_mode(SH1106_DC_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,SH1106_DC_PIN);
	GPIO_out_cfg(SH1106_DC_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,SH1106_DC_PIN);
	SH1106_DC_L();

	// Configure RST pin as push-pull output with pull-up
	GPIO_set_mode(SH1106_RST_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,SH1106_RST_PIN);
	GPIO_out_cfg(SH1106_RST_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,SH1106_RST_PIN);
	SH1106_RST_H();
}
*/

// Initialize SPI peripheral and SH1106 display
// note: SPI peripheral must be initialized before
void SH1106_Init(void) {
	// Hardware display reset
	SH1106_CS_H();
	SH1106_RST_L();
	SH1106_RST_H();
	SH1106_CS_L();

	// Initial display configuration

	// Set multiplex ratio (visible lines)
	SH1106_cmd_double(SH1106_CMD_SETMUX,0x3F); // 64MUX

	// Set display offset (offset of first line from the top of display)
	SH1106_cmd_double(SH1106_CMD_SETOFFS,0x00); // Offset: 0

	// Set display start line (first line displayed)
	SH1106_cmd(SH1106_CMD_STARTLINE | 0x00); // Start line: 0

	// Set segment re-map (X coordinate)
	SH1106_cmd(SH1106_CMD_SEG_NORM);

	// Set COM output scan direction (Y coordinate)
	SH1106_cmd(SH1106_CMD_COM_NORM);

	// Set COM pins hardware configuration
	// bit[4]: reset - sequential COM pin configuration
	//         set   - alternative COM pin configuration (reset value)
	// bit[5]: reset - disable COM left/right remap (reset value)
	//         set   - enable COM left/right remap
	SH1106_cmd_double(SH1106_CMD_COM_HW,0x12);

	uint8_t dis_charge = 0x00;
	uint8_t pre_charge = 0x00;

	SH1106_cmd_double(SH1106_CMD_CHARGE, dis_charge | (pre_charge << 4));

	// Set contrast control
	SH1106_cmd_double(SH1106_CMD_CONTRAST, 0x0F); // Contrast: middle level

	SH1106_cmd(0x30);

	// Disable entire display ON
	SH1106_cmd(SH1106_CMD_EDOFF); // Display follows RAM content

	// Disable display inversion
	SH1106_cmd(SH1106_CMD_INV_OFF); // Normal display mode

	// Set clock divide ratio and oscillator frequency
	// bits[3:0] defines the divide ratio of the display clocks (bits[3:0] + 1)
	// bits[7:4] set the oscillator frequency (Fosc), frequency increases with the value of these bits
	// 0xF0 value gives maximum frequency (maximum Fosc without divider)
	// 0x0F value gives minimum frequency (minimum Fosc divided by 16)
	// The higher display frequency decreases image flickering but increases current consumption and vice versa
	SH1106_cmd_double(SH1106_CMD_CLOCKDIV,0xF0);

	// Display ON
	SH1106_cmd(SH1106_CMD_DISP_ON); // Display enabled

	// Assert CS pin
	SH1106_CS_H();
}

// Set display contrast
// input:
//   contrast - new contrast value (0..255)
void SH1106_Contrast(uint8_t contrast) {
	SH1106_CS_L();
	SH1106_cmd_double(SH1106_CMD_CONTRAST,contrast);
	SH1106_CS_H();
}

// Set entire LCD pixels on or off
// input:
//   eon_state - new pixels state (one of LCD_ENTIRE_PIXELS_XXX values)
// note: LCD_ENTIRE_PIXELS_ON means what all pixels on display will be on
//       without regard of display memory contents
void SH1106_SetAllPixelsOn(uint8_t eon_state) {
	SH1106_CS_L();
	SH1106_cmd(eon_state ? SH1106_CMD_EDON : SH1106_CMD_EDOFF);
	SH1106_CS_H();
}

// Set display pixels inversion
// input:
//   inv_state - new state of display inversion (one of LCD_INVERT_XXX values)
// note: LCD_INVERT_ON means what all pixels on display will be inverted
void SH1106_SetInvert(uint8_t inv_state) {
	SH1106_CS_L();
	SH1106_cmd(inv_state ? SH1106_CMD_INV_ON : SH1106_CMD_INV_OFF);
	SH1106_CS_H();
}

// Toggle display on/off
// input:
//   disp_state - new display state (LCD_ON or LCD_OFF)
// note: doesn't affect the display memory
void SH1106_SetDisplayState(uint8_t disp_state) {
	SH1106_CS_L();
	SH1106_cmd(disp_state ? SH1106_CMD_DISP_ON : SH1106_CMD_DISP_OFF);
	SH1106_CS_H();
}

// Set X coordinate mapping (normal or mirrored)
// input:
//   x_map - new mapping of X coordinate (one of LCD_INVERT_XXX values)
// note: LCD_INVERT_OFF means normal SEG scan direction
// note: new setting will only affect subsequent data output
void SH1106_SetXDir(uint8_t x_map) {
	SH1106_CS_L();
	SH1106_cmd(x_map ? SH1106_CMD_SEG_INV : SH1106_CMD_SEG_NORM);
	SH1106_CS_H();
}

// Set Y coordinate mapping (normal or mirrored)
// input:
//   y_map - new mapping of Y coordinate (one of LCD_INVERT_XXX values)
// note: LCD_INVERT_OFF means normal COM scan direction
// note: new setting flip screen image immediately
void SH1106_SetYDir(uint8_t y_map) {
	SH1106_CS_L();
	SH1106_cmd(y_map ? SH1106_CMD_COM_INV : SH1106_CMD_COM_NORM);
	SH1106_CS_H();
}

// Set display orientation
// input:
//   orientation - new display orientation (one of LCD_ORIENT_XXX values)
// note: normal orientation is FPC on top of COG
// note: this setting specifies an orientation of display, not orientation of image
void SH1106_Orientation(uint8_t orientation) {
	// Configure display SEG/COM scan direction
	switch(orientation) {
		case LCD_ORIENT_CW:
			// Clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			SH1106_SetXDir(LCD_INVERT_ON);
			SH1106_SetYDir(LCD_INVERT_OFF);
			break;
		case LCD_ORIENT_CCW:
			// Counter-clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			SH1106_SetXDir(LCD_INVERT_OFF);
			SH1106_SetYDir(LCD_INVERT_ON);
			break;
		case LCD_ORIENT_180:
			// 180 degree rotation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			SH1106_SetXDir(LCD_INVERT_OFF);
			SH1106_SetYDir(LCD_INVERT_OFF);
			break;
		default:
			// Normal orientation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			SH1106_SetXDir(LCD_INVERT_ON);
			SH1106_SetYDir(LCD_INVERT_ON);
			break;
	}

	// Store orientation
	scr_orientation = orientation;
}


// Send vRAM buffer into display

void SH1106_Flush(void) {

	SH1106_CS_L();
	SH1106_DC_L();

	const uint32_t bits_h = SCR_H >> 3;

	uint8_t ram_pointer[] = {
		SH1106_CMD_COL_LOW | 0x02,
		SH1106_CMD_COL_HIGH,
		0
	};

	uint8_t* page_addr = &ram_pointer[2];

	for (register uint32_t page = 0; page < bits_h; ++page)
	{
		(*page_addr) = SH1106_CMD_PAGE_ADDR | page;

		HAL_SPI_Transmit(&SH1106_SPI_PORT,(uint8_t *)ram_pointer,sizeof(ram_pointer),1000);

		uint8_t* vram = &vRAM[page * SCR_W];

		SH1106_DC_H();
		HAL_SPI_Transmit(&SH1106_SPI_PORT,vram, SCR_W, 1000);

		SH1106_DC_L();
	}

	SH1106_CS_H();
}

// Fill vRAM memory with specified pattern
// input:
//   pattern - byte to fill vRAM buffer
void SH1106_Fill(uint8_t pattern) {
	uint16_t i;

	for (i = (SCR_W * SCR_H) >> 3; i--; ) {
		vRAM[i] = pattern;
	}
}

// Horizontal scroll setup
// input:
//   dir - scroll direction (one of LCD_SCROLL_XXX values)
//   start - start page address [0..7]
//   end - end page address [0..7], must be great or equal to start value
//   interval - time interval between scroll steps (one of LCD_SCROLL_IFXXX values)
void SH1106_ScrollHSetup(uint8_t dir, uint8_t start, uint8_t end, uint8_t interval) {
	SH1106_CS_L();
	SH1106_cmd((dir == LCD_SCROLL_RIGHT) ? SH1106_CMD_SCRL_HR : SH1106_CMD_SCRL_HL);
	SH1106_cmd(0x00); // dummy byte
	SH1106_cmd(start); // Start page address
	SH1106_cmd(interval); // Time interval between each scroll stop in terms of frame frequency
	SH1106_cmd(end); // End page address
	SH1106_cmd(0x00); // dummy byte
	SH1106_cmd(0xFF); // dummy byte
	SH1106_CS_H();
}

// Diagonal (vertical and horizontal) scroll setup
// input:
//   dir - horizontal scroll direction (one of LCD_SCROLL_XXX values)
//   start - start page address [0..7]
//   end - end page address [0..7], must be great or equal to start value
//   interval - time interval between scroll steps (one of LCD_SCROLL_IFXXX values)
//   voffs - vertical scrolling offset, this value specifies how many lines will
//           be scrolled vertically per one scroll step [1..63]
void SH1106_ScrollDSetup(uint8_t dir, uint8_t start, uint8_t end, uint8_t interval, uint8_t voffs) {
	SH1106_CS_L();
	SH1106_cmd((dir == LCD_SCROLL_RIGHT) ? SH1106_CMD_SCRL_VHR : SH1106_CMD_SCRL_VHL);
	SH1106_cmd(0x00); // dummy byte
	SH1106_cmd(start); // Start page address
	SH1106_cmd(interval); // Time interval between each scroll stop in terms of frame frequency
	SH1106_cmd(end); // End page address
	SH1106_cmd(voffs); // Vertical scrolling offset
	SH1106_CS_H();
}

// Activate scrolling
// note: this function must be called only after scroll setup
// note: changing of video RAM contents and scroll parameters are prohibited
//       after the scrolling is activated
void SH1106_ScrollStart(void) {
	SH1106_CS_L();
	SH1106_cmd(SH1106_CMD_SCRL_ACT);
	SH1106_CS_H();
}

// Deactivate scrolling
// note: after calling this function the graphics RAM data needs to be rewritten
void SH1106_ScrollStop(void) {
	SH1106_CS_L();
	SH1106_cmd(SH1106_CMD_SCRL_STOP);
	SH1106_CS_H();
}

// Set pixel in vRAM buffer
// input:
//   X, Y - pixel coordinates
//   Mode - pixel mode (one of LCD_PXXX values)
#if (SH1106_OPT_PIXEL)
__attribute__((always_inline)) void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode) {
#else
void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode) {
#endif // SH1106_OPT_PIXEL
	register uint32_t offset;
	register uint32_t bpos;

	// Offset of pixel in the vRAM array must be computed by formula ((Y >> 3) * SCR_W) + X
	// Since screen is 128 pixel width the formula can be simplified to ((Y >> 3) << 7) + X
	// For 90 degree rotation X and Y must be swapped
	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		offset = ((X >> 3) << 7) + Y;
		bpos   = X & 0x07;
	} else {
		offset = ((Y >> 3) << 7) + X;
		bpos   = Y & 0x07;
	}

	// Return if offset went out outside of vRAM
	if (offset > ((SCR_W * SCR_H) >> 3)) {
		return;
	}

#if (SH1106_USE_BITBAND)
	switch (Mode) {
		case LCD_PRES:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2))  = 0;
			break;
		case LCD_PINV:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2)) ^= 1;
			break;
		default:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2))  = 1;
			break;
	}
#else // (SH1106_USE_BITBAND)
	switch (Mode) {
		case LCD_PRES:
			vRAM[offset] &= ~(1 << bpos);
			break;
		case LCD_PINV:
			vRAM[offset] ^=  (1 << bpos);
			break;
		default:
			vRAM[offset] |=  (1 << bpos);
			break;
	}
#endif // SH1106_USE_BITBAND
}

// Optimized draw horizontal line (without regard of screen rotation)
// input:
//   X - horizontal coordinate of line start
//   Y - vertical coordinate
//   W - line width
static void LCD_HLineInt(uint8_t X, uint8_t Y, uint8_t W) {
	uint8_t *ptr;
	uint8_t mask;

	// Pointer to the first byte of line in video buffer
	// This is optimized formula, original is "((Y >> 3) * SCR_W) + X"
	ptr = &vRAM[((Y >> 3) << 7)] + X;

	// Mask bit for pixel in byte
	mask = 1 << (Y & 0x07);

	// Draw line
	switch (LCD_PixelMode) {
		case LCD_PRES:
			mask = ~mask;
			while (W--) *ptr++ &= mask;
			break;
		case LCD_PINV:
			while (W--) *ptr++ ^= mask;
			break;
		default:
			while (W--) *ptr++ |= mask;
			break;
	}
}

// Optimized draw vertical line (without regard of screen rotation)
// input:
//   X - horizontal coordinate
//   Y - vertical coordinate
//   H - line length
static void LCD_VLineInt(uint8_t X, uint8_t Y, uint8_t H) {
	uint8_t *ptr;
	uint8_t mask;
	uint8_t modulo;

	// Pointer to the first byte of line in video buffer
	// This is optimized formula, original is "((Y >> 3) * SCR_W) + X"
	ptr = &vRAM[((Y >> 3) << 7)] + X;

	// First partial byte?
	modulo = (Y & 0x07);
	if (modulo) {
		// Get bit mask for first partial byte from lookup table
		modulo = 8 - modulo;
		mask = LUT_FB[modulo];

		// Trim mask if line is will not go out from a current byte
		if (modulo > H) mask &= (0xFF >> (modulo - H));

		// Modify bits in first byte of line
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr &= ~mask;
				break;
			case LCD_PINV:
				*ptr ^=  mask;
				break;
			default:
				*ptr |=  mask;
				break;
		}

		// Return if line is over
		if (modulo > H) return;

		// Shift pointer to the next byte in line and decrease line height counter
		ptr += SCR_W;
		H   -= modulo;
	}

	// Fill solid bytes
	if (H > 7) {
		// Separate cycle for each case of pixel mode (to improve performance)
		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					*ptr = 0x00;
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
			case LCD_PINV:
				do {
					*ptr = ~(*ptr);
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
			default:
				do {
					*ptr = 0xFF;
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
		}
	}

	// Last partial byte?
	if (H) {
		// Get bit mask for last partial byte from lookup table
		modulo = (H & 0x07);
		mask   = LUT_LB[modulo];

		// Modify bits in last byte of line
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr &= ~mask;
				break;
			case LCD_PINV:
				*ptr ^=  mask;
				break;
			default:
				*ptr |=  mask;
				break;
		}
	}
}

// Draw horizontal line
// input:
//   X1, X2 - left and right horizontal coordinates
//   Y - vertical coordinate
void LCD_HLine(uint8_t X1, uint8_t X2, uint8_t Y) {
	uint8_t X,W;

	if (X1 > X2) {
		X = X2; W = X1 - X2;
	} else {
		X = X1; W = X2 - X1;
	}
	W++;

	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		LCD_VLineInt(Y,X,W);
	} else {
		LCD_HLineInt(X,Y,W);
	}
}

// Draw vertical line
// input:
//   X - horizontal coordinate
//   Y1,Y2 - top and bottom vertical coordinates (Y1 must be less than Y2)
void LCD_VLine(uint8_t X, uint8_t Y1, uint8_t Y2) {
	uint8_t Y,H;

	if (Y1 > Y2) {
		Y = Y2; H = Y1 - Y2;
	} else {
		Y = Y1; H = Y2 - Y1;
	}
	H++;

	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		LCD_HLineInt(Y,X,H);
	} else {
		LCD_VLineInt(X,Y,H);
	}
}

// Draw rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2) {
	LCD_HLine(X1,X2,Y1);
	LCD_HLine(X1,X2,Y2);
	LCD_VLine(X1,Y1 + 1,Y2 - 1);
	LCD_VLine(X2,Y1 + 1,Y2 - 1);
}

// Draw filled rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2) {
	uint8_t Z,E,T,L;

	// Fill rectangle by vertical lines is most optimal, therefore calculate coordinates
	// with regard of screen rotation
	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		if (X1 > X2) {
			T = X2; L = X1 - X2;
		} else {
			T = X1; L = X2 - X1;
		}

		if (Y1 > Y2) {
			Z = Y1; E = Y2;
		} else {
			Z = Y2; E = Y1;
		}
	} else {
		if (Y1 > Y2) {
			T = Y2; L = Y1 - Y2;
		} else {
			T = Y1; L = Y2 - Y1;
		}

		if (X1 > X2) {
			Z = X1; E = X2;
		} else {
			Z = X2; E = X1;
		}
	}
	L++;

	// Fill a rectangle
	do {
		LCD_VLineInt(Z,T,L);
	} while (Z-- > E);
}

// Draw line
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2) {
	int16_t dX = X2 - X1;
	int16_t dY = Y2 - Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		LCD_VLine(X1,Y1,Y2);

		return;
	}
	if (dY == 0) {
		LCD_HLine(X1,X2,Y1);

		return;
	}

	dX *= dXsym;
	dY *= dYsym;
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			LCD_Pixel(X1,Y1,LCD_PixelMode);
			X1 += dXsym;
			if (di < 0) {
				di += dY2;
			} else {
				di += dY2 - dX2;
				Y1 += dYsym;
			}
		}
	} else {
		di = dX2 - dY;
		while (Y1 != Y2) {
			LCD_Pixel(X1,Y1,LCD_PixelMode);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	LCD_Pixel(X1,Y1,LCD_PixelMode);
}

// Draw circle
// input:
//   Xc,Yc - circle center coordinates
//   R - circle radius
void LCD_Circle(int16_t Xc, int16_t Yc, uint8_t R) {
	int16_t err = 1 - R;
	int16_t dx  = 0;
	int16_t dy  = -2 * R;
	int16_t x   = 0;
	int16_t y   = R;
	// Screen width and height for less calculations
	int16_t sh  = scr_height - 1;
	int16_t sw  = scr_width  - 1;

	while (x < y) {
		if (err >= 0) {
			dy  += 2;
			err += dy;
			y--;
		}
		dx  += 2;
		err += dx + 1;
		x++;

		// Draw eight pixels of each octant
		if (Xc + x < sw) {
			if (Yc + y < sh) LCD_Pixel(Xc + x,Yc + y,LCD_PixelMode);
			if (Yc - y > -1) LCD_Pixel(Xc + x,Yc - y,LCD_PixelMode);
		}
		if (Xc - x > -1) {
			if (Yc + y < sh) LCD_Pixel(Xc - x,Yc + y,LCD_PixelMode);
			if (Yc - y > -1) LCD_Pixel(Xc - x,Yc - y,LCD_PixelMode);
		}
		if (Xc + y < sw) {
			if (Yc + x < sh) LCD_Pixel(Xc + y,Yc + x,LCD_PixelMode);
			if (Yc - x > -1) LCD_Pixel(Xc + y,Yc - x,LCD_PixelMode);
		}
		if (Xc - y > -1) {
			if (Yc + x < sh) LCD_Pixel(Xc - y,Yc + x,LCD_PixelMode);
			if (Yc - x > -1) LCD_Pixel(Xc - y,Yc - x,LCD_PixelMode);
		}
	}

	// Vertical and horizontal points
	if (Xc + R < sw) LCD_Pixel(Xc + R,Yc,LCD_PixelMode);
	if (Xc - R > -1) LCD_Pixel(Xc - R,Yc,LCD_PixelMode);
	if (Yc + R < sh) LCD_Pixel(Xc,Yc + R,LCD_PixelMode);
	if (Yc - R > -1) LCD_Pixel(Xc,Yc - R,LCD_PixelMode);
}

// Draw ellipse
// input:
//   Xc,Yc - coordinates of center of the ellipse
//   Ra,Rb - horizontal and vertical radiuses
void LCD_Ellipse(uint16_t Xc, uint16_t Yc, uint16_t Ra, uint16_t Rb) {
	int16_t x  = 0;
	int16_t y  = Rb;
	int32_t A2 = Ra * Ra;
	int32_t B2 = Rb * Rb;
	int32_t C1 = -((A2 >> 2) + (Ra & 0x01) + B2);
	int32_t C2 = -((B2 >> 2) + (Rb & 0x01) + A2);
	int32_t C3 = -((B2 >> 2) + (Rb & 0x01));
	int32_t t  = -A2 * y;
	int32_t dX = B2 * x * 2;
	int32_t dY = -A2 * y * 2;
	int32_t dXt2 = B2 * 2;
	int32_t dYt2 = A2 * 2;
	// Screen width and height for less calculations
	int16_t sh  = scr_height - 1;
	int16_t sw  = scr_width  - 1;

	while ((y >= 0) && (x <= Ra)) {
		if ((Xc + x < sw) && (Yc + y < sh)) {
			LCD_Pixel(Xc + x,Yc + y,LCD_PixelMode);
		}
		if (x || y) {
			if ((Xc - x > -1) && (Yc - y > -1)) {
				LCD_Pixel(Xc - x,Yc - y,LCD_PixelMode);
			}
		}
		if (x && y) {
			if ((Xc + x < sw) && (Yc - y > - 1)) {
				LCD_Pixel(Xc + x,Yc - y,LCD_PixelMode);
			}
			if ((Xc - x > -1) && (Yc + y < sh)) {
				LCD_Pixel(Xc - x,Yc + y,LCD_PixelMode);
			}
		}

		if ((t + x*B2 <= C1) || (t + y*A2 <= C3)) {
			dX += dXt2;
			t  += dX;
			x++;
		} else if (t - y*A2 > C2) {
			dY += dYt2;
			t  += dY;
			y--;
		} else {
			dX += dXt2;
			dY += dYt2;
			t  += dX;
			t  += dY;
			x++;
			y--;
		}
	}
}

// Draw a single character
// input:
//   X,Y - character top left corner coordinates
//   Char - character to be drawn
//   Font - pointer to font
// return: character width in pixels
uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;
	const uint8_t *pCh;

	// If the specified character code is out of bounds should substitute the code of the "unknown" character
	if (Char < Font->font_MinChar || Char > Font->font_MaxChar) Char = Font->font_UnknownChar;

	// Pointer to the first byte of character in font data array
	pCh = &Font->font_Data[(Char - Font->font_MinChar) * Font->font_BPC];

	// Draw character
	if (Font->font_Scan == FONT_V) {
		// Vertical pixels order
		if (Font->font_Height < 9) {
			// Height is 8 pixels or less (one byte per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
					tmpCh >>= 1;
					pY++;
				}
				pX++;
			}
		} else {
			// Height is more than 8 pixels (several bytes per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				while (pY < Y + Font->font_Height) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
							tmpCh >>= 1;
							if (tmpCh) {
								pY++;
								bL--;
							} else {
								pY += bL;
								break;
							}
						}
					} else {
						pY += bL;
					}
				}
				pX++;
			}
		}
	} else {
		// Horizontal pixels order
		if (Font->font_Width < 9) {
			// Width is 8 pixels or less (one byte per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
					tmpCh >>= 1;
					pX++;
				}
				pY++;
			}
		} else {
			// Width is more than 8 pixels (several bytes per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				while (pX < X + Font->font_Width) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
							tmpCh >>= 1;
							if (tmpCh) {
								pX++;
								bL--;
							} else {
								pX += bL;
								break;
							}
						}
					} else {
						pX += bL;
					}
				}
				pY++;
			}
		}
	}

	return Font->font_Width + 1;
}

// Draw string
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t LCD_PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	uint8_t pX = X;
	uint8_t eX = scr_width - Font->font_Width - 1;

	while (*str) {
		pX += LCD_PutChar(pX,Y,*str++,Font);
		if (pX > eX) break;
	}

	return (pX - X);
}

// Draw string with line feed at screen edge
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t LCD_PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	uint8_t strLen = 0;

    while (*str) {
        LCD_PutChar(X,Y,*str++,Font);
        if (X < scr_width - Font->font_Width - 1) {
        	X += Font->font_Width + 1;
        } else if (Y < scr_height - Font->font_Height - 1) {
        	X = 0; Y += Font->font_Height;
        } else {
        	X = 0; Y = 0;
        }
        strLen++;
    };

    return strLen * (Font->font_Width + 1);
}

// Draw signed integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - signed integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);
	if (neg) *pStr++ = '-';

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw unsigned integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint8_t pX = X;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw signed integer value with decimal point
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   decimals - number of digits after decimal point
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;
	uint8_t strLen = 0;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do {
		*pStr++ = (num % 10) + '0';
		strLen++;
	} while (num /= 10);

	// Add leading zeroes
	if (strLen <= decimals) {
		while (strLen <= decimals) {
			*pStr++ = '0';
			strLen++;
		}
	}

	// Minus sign?
	if (neg) {
		*pStr++ = '-';
		strLen++;
	}

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX,Y,*pStr,Font);
		if (decimals && (--strLen == decimals)) {
			// Draw decimal point
			LCD_Rect(pX,Y + Font->font_Height - 2,pX + 1,Y + Font->font_Height - 1);
			pX += 3;
		}
	}

	return (pX - X);
}

// Draw signed integer value with leading zeros
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   digits - minimal number of length (e.g. num=35, digits=5 --> 00035)
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;
	uint8_t strLen = 0;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do {
		*pStr++ = (num % 10) + '0';
		strLen++;
	} while (num /= 10);

	// Add leading zeroes
	if (strLen < digits) while (strLen++ < digits) *pStr++ = '0';

	// Minus sign?
	if (neg) *pStr++ = '-';

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw integer as hexadecimal
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint8_t pX = X;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	do {
		*pStr = (num % 0x10) + '0';
		if (*pStr > '9') *pStr += 7;
		pStr++;
	} while (num /= 0x10);

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw monochrome bitmap
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// note: each '1' bit in the bitmap will be drawn as a pixel
//       each '0' bit in the will not be drawn (transparent bitmap)
// bitmap: one byte per 8 vertical pixels, LSB top, truncate bottom bits
void LCD_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;

	pY = Y;
	while (pY < Y + H) {
		pX = X;
		while (pX < X + W) {
			bL = 0;
			tmpCh = *pBMP++;
			if (tmpCh) {
				while (bL < 8) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY + bL,LCD_PixelMode);
					tmpCh >>= 1;
					if (tmpCh) {
						bL++;
					} else {
						pX++;
						break;
					}
				}
			} else {
				pX++;
			}
		}
		pY += 8;
	}
}

void LCD_DrawBitmapFullscreen(const uint8_t* pBMP)
{
	memcpy(vRAM, pBMP, (SCR_W * SCR_H) >> 3);
}
