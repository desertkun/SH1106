#ifndef __SH1106_H
#define __SH1106_H

#include "stm32f1xx_hal.h"

// SH1106 display connection:
//   PB12 --> CS
//   PB14 --> RES
//   PC6  --> DC
//   PB13 --> CLK
//   PB15 --> MOSI


// Use bit-banding to draw pixel
//   0 - use logic operations to set pixel color
//   1 - use bit-banding to set pixel color
#define SH1106_USE_BITBAND   1

// Pixel set function definition
//   0 - call pixel function (less code size in cost of speed)
//   1 - inline pixel function (higher speed in cost of code size)
#define SH1106_OPT_PIXEL     1

// DMA usage
//   0 - DMA is not used
//   1 - compile functions for DMA transfer VRAM to display
#define SH1106_USE_DMA       0


// SH1106 HAL

// SPI port
#define SH1106_SPI_PORT      hspi2

// GPIO peripherals
#define SH1106_GPIO_PERIPH   (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN)

// SH1106 RS/A0 (Data/Command select) pin (PC6)
#define SH1106_DC_PORT       GPIOC
#define SH1106_DC_PIN        GPIO_PIN_6
#define SH1106_DC_H()        HAL_GPIO_WritePin(SH1106_DC_PORT, SH1106_DC_PIN,GPIO_PIN_SET)
#define SH1106_DC_L()        HAL_GPIO_WritePin(SH1106_DC_PORT, SH1106_DC_PIN,GPIO_PIN_RESET)

// SH1106 RST (Reset) pin (PB14)
#define SH1106_RST_PORT      GPIOB
#define SH1106_RST_PIN       GPIO_PIN_14
#define SH1106_RST_H()       HAL_GPIO_WritePin(SH1106_RST_PORT, SH1106_RST_PIN,GPIO_PIN_SET)
#define SH1106_RST_L()       HAL_GPIO_WritePin(SH1106_RST_PORT, SH1106_RST_PIN,GPIO_PIN_RESET)

// SH1106 CS (Chip Select) pin (PB12)
#define SH1106_CS_PORT       GPIOB
#define SH1106_CS_PIN        GPIO_PIN_12
#define SH1106_CS_H()        HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN,GPIO_PIN_SET)
#define SH1106_CS_L()        HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN,GPIO_PIN_RESET)


// Screen dimensions
#define SCR_W                 (uint8_t)128 // width
#define SCR_H                 (uint8_t)64  // height


// SH1106 command definitions
#define SH1106_CMD_SETMUX    (uint8_t)0xA8 // Set multiplex ratio (N, number of lines active on display)
#define SH1106_CMD_SETOFFS   (uint8_t)0xD3 // Set display offset
#define SH1106_CMD_STARTLINE (uint8_t)0x40 // Set display start line
#define SH1106_CMD_SEG_NORM  (uint8_t)0xA0 // Column 0 is mapped to SEG0 (X coordinate normal)
#define SH1106_CMD_SEG_INV   (uint8_t)0xA1 // Column 127 is mapped to SEG0 (X coordinate inverted)
#define SH1106_CMD_COM_NORM  (uint8_t)0xC0 // Scan from COM0 to COM[N-1] (N - mux ratio, Y coordinate normal)
#define SH1106_CMD_COM_INV   (uint8_t)0xC8 // Scan from COM[N-1] to COM0 (N - mux ratio, Y coordinate inverted)
#define SH1106_CMD_COM_HW    (uint8_t)0xDA // Set COM pins hardware configuration
#define SH1106_CMD_CONTRAST  (uint8_t)0x81 // Contrast control
#define SH1106_CMD_EDON      (uint8_t)0xA5 // Entire display ON enabled (all pixels on, RAM content ignored)
#define SH1106_CMD_EDOFF     (uint8_t)0xA4 // Entire display ON disabled (output follows RAM content)
#define SH1106_CMD_INV_OFF   (uint8_t)0xA6 // Entire display inversion OFF (normal display)
#define SH1106_CMD_INV_ON    (uint8_t)0xA7 // Entire display inversion ON (all pixels inverted)
#define SH1106_CMD_CLOCKDIV  (uint8_t)0xD5 // Set display clock divide ratio/oscillator frequency
#define SH1106_CMD_DISP_ON   (uint8_t)0xAF // Display ON
#define SH1106_CMD_DISP_OFF  (uint8_t)0xAE // Display OFF (sleep mode)

#define SH1106_CMD_COL_LOW   (uint8_t)0x00 // Set Lower Column Address
#define SH1106_CMD_COL_HIGH  (uint8_t)0x10 // Set Higher Column Address
#define SH1106_CMD_PAGE_ADDR (uint8_t)0xB0 // Set Page Address

#define SH1106_CMD_VCOMH     (uint8_t)0xDB // Set Vcomh deselect level
#define SH1106_CMD_SCRL_HR   (uint8_t)0x26 // Setup continuous horizontal scroll right
#define SH1106_CMD_SCRL_HL   (uint8_t)0x27 // Setup continuous horizontal scroll left
#define SH1106_CMD_SCRL_VHR  (uint8_t)0x29 // Setup continuous vertical and horizontal scroll right
#define SH1106_CMD_SCRL_VHL  (uint8_t)0x2A // Setup continuous vertical and horizontal scroll left
#define SH1106_CMD_SCRL_STOP (uint8_t)0x2E // Deactivate scroll
#define SH1106_CMD_SCRL_ACT  (uint8_t)0x2F // Activate scroll


// Entire display on/off enumeration
enum {
	LCD_ENTIRE_PIXELS_OFF = 0,
	LCD_ENTIRE_PIXELS_ON  = !LCD_ENTIRE_PIXELS_OFF
};

// Display pixels inversion enumeration
enum {
	LCD_INVERT_OFF = 0,
	LCD_INVERT_ON  = !LCD_INVERT_OFF
};

// Display ON/OFF enumeration
enum {
	LCD_OFF = 0,
	LCD_ON  = !LCD_OFF
};

// Screen orientation enumeration
enum {
	LCD_ORIENT_NORMAL = 0, // No rotation
	LCD_ORIENT_CW     = 1, // Clockwise rotation
	LCD_ORIENT_CCW    = 2, // Counter-clockwise rotation
	LCD_ORIENT_180    = 3  // 180 degrees rotation
};

// Screen horizontal scroll direction enumeration
enum {
	LCD_SCROLL_RIGHT = 0, // Scroll right
	LCD_SCROLL_LEFT  = 1  // Scroll left
};

// Screen scroll interval enumeration
enum {
	LCD_SCROLL_IF2   = 0x07, // 2 frames
	LCD_SCROLL_IF3   = 0x04, // 3 frames
	LCD_SCROLL_IF4   = 0x05, // 4 frames
	LCD_SCROLL_IF5   = 0x00, // 5 frames
	LCD_SCROLL_IF25  = 0x06, // 25 frames
	LCD_SCROLL_IF64  = 0x01, // 64 frames
	LCD_SCROLL_IF128 = 0x02, // 128 frames
	LCD_SCROLL_IF256 = 0x03  // 256 frames
};

// Pixel draw mode
enum {
	LCD_PSET = 0x00, // Set pixel
	LCD_PRES = 0x01, // Reset pixel
	LCD_PINV = 0x02  // Invert pixel
};

// Font structure scan lines enumeration
enum {
	FONT_V = (uint8_t)0,        // Vertical font scan lines
	FONT_H = (uint8_t)(!FONT_V) // Horizontal font scan lines
};


// Font descriptor
typedef struct {
	uint8_t font_Width;       // Width of character
	uint8_t font_Height;      // Height of character
	uint8_t font_BPC;         // Bytes for one character
	uint8_t font_Scan;        // Font scan lines behavior
	uint8_t font_MinChar;     // Code of the first known symbol
	uint8_t font_MaxChar;     // Code of the last known symbol
	uint8_t font_UnknownChar; // Code of the unknown symbol
	uint8_t font_Data[];      // Font data
} Font_TypeDef;


// Public variables
extern uint16_t scr_width;
extern uint16_t scr_height;
extern uint8_t LCD_PixelMode;

extern SPI_HandleTypeDef SH1106_SPI_PORT;

// Function prototypes
//void SH1106_InitGPIO(void);
void SH1106_Init(void);

void SH1106_Contrast(uint8_t contrast);
void SH1106_SetAllPixelsOn(uint8_t eon_state);
void SH1106_SetInvert(uint8_t inv_state);
void SH1106_SetDisplayState(uint8_t disp_state);
void SH1106_SetXDir(uint8_t x_map);
void SH1106_SetYDir(uint8_t y_map);
void SH1106_Orientation(uint8_t orientation);

void SH1106_Flush(void);
#if (SH1106_USE_DMA)
void SH1106_Flush_DMA(void);
#endif // SH1106_USE_DMA

void SH1106_Fill(uint8_t pattern);

void SH1106_ScrollHSetup(uint8_t dir, uint8_t start, uint8_t end, uint8_t interval);
void SH1106_ScrollDSetup(uint8_t dir, uint8_t start, uint8_t end, uint8_t interval, uint8_t voffs);
void SH1106_ScrollStart(void);
void SH1106_ScrollStop(void);

#if (SH1106_OPT_PIXEL)
inline void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode);
#else
void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode);
#endif // SH1106_OPT_PIXEL

void LCD_HLine(uint8_t X1, uint8_t X2, uint8_t Y);
void LCD_VLine(uint8_t X, uint8_t Y1, uint8_t Y2);
void LCD_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void LCD_Circle(int16_t X, int16_t Y, uint8_t R);
void LCD_Ellipse(uint16_t Xc, uint16_t Yc, uint16_t Ra, uint16_t Rb);

uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font);
uint16_t LCD_PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint16_t LCD_PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint8_t LCD_PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font);
uint8_t LCD_PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);
uint8_t LCD_PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font);
uint8_t LCD_PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font);
uint8_t LCD_PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);

void LCD_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);

#endif // __SH1106_H
