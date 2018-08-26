#include <stdio.h>
#include <time.h>

#include "mgos.h"
#include "mgos_config.h"
#include "mgos_spi.h"

#include "xpt2046.h"

/***************************************************************************************
***************************************************************************************/
//#define USE_GLOBAL_SPI 1
//#define USE_GLOBAL_SPI 0

static struct mgos_spi *xpt2046_spi;
static struct mgos_spi_txn xpt2046_txn;
#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
static struct mgos_config_spi xpt2046_bus_cfg;
#endif

uint32_t tp_calx = TP_CALX_XPT2046;  //??
uint32_t tp_caly = TP_CALY_XPT2046;

struct mgos_xpt2046_event_data xpt_last_touch;


static enum mgos_xpt2046_rotation_t _lcd_orientation = XPT2046_PORTRAIT;
static mgos_xpt2046_event_t xpt_event_handler = NULL;
static bool is_touching = false;
static uint16_t s_max_x = 240;
static uint16_t s_max_y = 320;
//******xpt2046 notes *******
/*
Control Bits in the Control Byte
(MSB) 														(LSB)
Bit 7 	Bit 6 	Bit 5 	Bit 4 	Bit 3 	Bit 2 		Bit 1 	Bit 0
	S 	A2 		A1 		A0 		MODE 	SER/DFR 	PD1 	PD0

Bit 0 y 1 ----	(PD1 - PD0) select the power- down mode

	class XPT2046(object):
	
	StartBit = 0b10000000 
	
	class ChannelSelect(object):
		X_POSITION 		= 0b01010000
		Y_POSITION 		= 0b00010000
		Z1_POSITION 	= 0b00110000
		Z2_POSITION 	= 0b01000000
		TEMPERATURE_0 	= 0b00000000
		TEMPERATURE_1 	= 0b01110000
		BATTERY_VOLTAGE = 0b00100000
		AUXILIARY 		= 0b01100000

	class ConversionSelect(object):
		_8_BIT  		= 0b00001000				//bit mode define la conversion de 8 o 12 bits
		_12_BIT 		= 0b00000000
		

		
				//0xB0 = 0b10110000    //leemos Z1 ???  ok***
				//0xD0 = 0b11010000    //leemos x ???  ok
				//0x90 = 0b10010000    //leemos y ???  ok
				//si ponemos el bit 0 y 1 a 0 	Power-down between conversions. When each
											conversion is finished, the converter enters a low
											power mode. At the start of the next conversion,
											the device instantly powers up to full power.
				//si ponemos el bit 3 a 0 estamos leyendo 12 bits

*/
//******xpt2046 notes *******

// ============= Touch panel functions =========================================


// ***mv *sustitucion de la funcion de lectura del bus SPI*****
// ***mv le cambiamos el nombre para mantenerla provisionalmente
// get 16-bit data from touch controller for specified type
// ** Touch device must already be selected **
//----------------------------------------
//**mv static int xpt2046_read_data(uint8_t type)
//**mv {
//**mv 	int res;
//**mv 	uint8_t rxbuf[2];
//**mv 
//**mv 	xpt2046_txn.hd.tx_len = 1;
//**mv 	xpt2046_txn.hd.tx_data = &type;
//**mv 	xpt2046_txn.hd.dummy_len = 0;
//**mv 	xpt2046_txn.hd.rx_len = 2;
//**mv 	xpt2046_txn.hd.rx_data = rxbuf;
//**mv 
//**mv 	if (!mgos_spi_run_txn(xpt2046_spi, false /* full_duplex */, &xpt2046_txn)) {
//**mv 		LOG(LL_ERROR, ("SPI transaction failed"));
//**mv 		return -1;
//**mv 	}

//**mv 	res = (rxbuf[0] << 8) | rxbuf[1];
//**mv   	//LOG(LL_INFO, ("****xpt2046_read_data linea_1  type:%d res:%d", type, res));
//**mv 
//**mv   	//LOG(LL_INFO, ("****xpt2046_read_data linea_1 res:%d", res));
//**mv 
//**mv   	//LOG(LL_INFO, ("res:"));
//**mv 
//**mv     return res;
//**mv }

// ***mv *sustitucion de la funcion de lectura del bus SPI*****

//**************nueva************************

//static uint8_t stmpe610_spi_read_register(uint8_t reg) {
//pendiente cambiar el tipo devuelto de uint8_t a int
static int xpt2046_read_data(uint8_t type) {
  struct mgos_spi *spi = mgos_spi_get_global();
  if (!spi) {
    LOG(LL_ERROR, ("Cannot get global SPI bus"));
    return 0;
  }

	//StartBit = 0b10000000    //control byte
	//  | Binary OR Operator copies a bit if it exists in either operand.
	//0x80 = 0b10000000
	
	//0xB0 = 0b10110000    //leemos Z ???
	//0xD0 = 0b11010000    //leemos x ???
	//0x90 = 0b10010000    //leemos y ???

  uint8_t tx_data = 0x80 | type;  //0x80 or type pone a 1 el bit de start siempre
  //uint8_t tx_data = type;
  uint8_t rx_data;
  int res;
  uint8_t rxbuf[2];
//**mv
  struct mgos_spi_txn txn = {
//      .cs = mgos_sys_config_get_stmpe610_cs_index(),
      .cs = mgos_sys_config_get_xpt2046_cs_index(),
      .mode = 0,
		/* mode, 0-3. This controls clock phase and polarity. */
      //.mode = 3,
      //.freq = 1000000,
      .freq = 100000,
  };
  txn.hd.tx_len = 1;
  //txn.hd.tx_data = &tx_data;
  txn.hd.tx_data = &tx_data;
  //txn.hd.tx_data = &type;
  txn.hd.dummy_len = 0;
//  txn.hd.rx_len = 1;
  txn.hd.rx_len = 2;
//  txn.hd.rx_data = &rx_data;
//  txn.hd.rx_data = &rx_data;
  //txn.hd.rx_data = &rxbuf;
  txn.hd.rx_data = rxbuf;

//**mv 	if (!mgos_spi_run_txn(xpt2046_spi, false /* full_duplex */, &xpt2046_txn)) {
//**mv 		LOG(LL_ERROR, ("SPI transaction failed"));
//**mv 		return -1;
//**mv 	}

 // if (!mgos_spi_run_txn(spi, false, &txn)) {
  if (!mgos_spi_run_txn(spi, false, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return 0;
  }
	res = (rxbuf[0] << 8) | rxbuf[1];
  // pendiente de convertir rx_data de uint8_t a int
  //rx_data = 34;
  //res = 95;
  //return rx_data;
   return res;
}

//**************fin nueva********************

//-------------------------------------------------------
static int xpt2046_get_touch_data(uint8_t type, int samples)
{
  	//LOG(LL_INFO, ("****xpt2046_get_touch_data linea_127  type:%d samples:%d", type, samples));

	if (xpt2046_spi == (void *)NULL) return 0;

	int n, result, val = 0;
	int avg = 0;
	uint32_t i = 0;
	uint32_t minval, maxval, dif=0;

    if (samples < 3) samples = 1;
    if (samples > 18) samples = 18;

    // one dummy read
    result = xpt2046_read_data(type);
    avg = result >> 3;

    // read data
	while (i < 10) {
    	minval = 5000;
    	maxval = 0;
		// get values
		for (n=0;n<samples;n++) {
		    result = xpt2046_read_data(type) >> 3;
		    avg = ((avg * 3) + result) / 4;

			if (result < 0) break;

//			vbuf[n] = result;
			if (result < minval) minval = result;
			if (result > maxval) maxval = result;
		}
		if (result < 0) break;
		dif = maxval - minval;
		if (dif < 40) break;
		i++;
    }

	if (result < 0) return -1;
	val = avg;

  	//LOG(LL_INFO, ("****xpt2046_get_touch_data linea_127  type:%d samples:%d resultado val:%d", type, samples, val));
  	//LOG(LL_INFO, ("****xpt2046_get_touch_data linea_167 resultado val:%d", val));
	
    return val;
}


//=============================================
//
int xpt2046_read_touch(int *x, int* y, int* z)
{
	int value = -1, res=0;
    int X=0, Y=0, Z=0;

	if (xpt2046_spi == (void *)NULL) return 0;

//	if (spi_lobo_device_select(xpt0246_spi, 0) != ESP_OK) return 0;

	//0xB0 = 0b10110000    //leemos Z ???
    value = xpt2046_get_touch_data(0xB0, 3);  // Z; pressure; touch detect
  	//LOG(LL_INFO, ("****xpt2046_read_touch linea_1 get_touch_data(0xB0, 3) at Z:%d", value));
	Z = value;

	//if (value <= 8)  goto exit;
	//if (value <= 50) 
	if (value <= 20) 
	{
		goto exit;
	}
	
	// touch panel pressed
	//0xD0 = 0b11010000    //leemos x ???
	value = xpt2046_get_touch_data(0xD0, 10);
  	//LOG(LL_INFO, ("****xpt2046_read_touch linea_2 get_touch_data(0xD0, 10) at x value:%d", value));

	if (value < 0)  goto exit;

	X = value;

	//0x90 = 0b10010000    //leemos y ???
	value = xpt2046_get_touch_data(0x90, 10);
  	//LOG(LL_INFO, ("****xpt2046_read_touch linea_3 get_touch_data(0x90, 10) at y value:%d", value));

	if (value < 0)  goto exit;
		
	Y = value;
	res = 1;


exit:
//	spi_lobo_device_deselect(xpt0246_spi);

   	if (value == 0) 
	{
		//LOG(LL_INFO, ("****xpt2046_read_touch linea_4  exit value <=0 ó <=50 at X:%d, Y:%d Z:%d", X, Y, Z));
		return 0;
	}
	*x = X;
	*y = Y;
	*z = Z;
	//LOG(LL_INFO, ("****xpt2046_read_touch linea_5 ok  at X:%d, Y:%d Z:%d", X, Y, Z));

	return res;
}



static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x<in_min) x=in_min;
	if (x>in_max) x=in_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static void xpt2046_map_rotation(uint16_t x, uint16_t y, uint16_t *x_out, uint16_t *y_out)
{
	//s_max_x = x;  //tama?o en pixel de la pantalla en eje x declarado en linea 471
	//s_max_y = y;  //tama?o en pixel de la pantalla en eje y declarado en linea 472
	//*x_out  //valor de la coordenada x retornado al programa
	//*y_out  //valor de la coordenada y retornado al programa
	//x valor a convertir del eje x
	//y valor a convertir del eje y
	//map(y, ymin, xmax, 0, s_max_x);  //func definida en linea 285 
	
	//const int xmax = (tp_calx >> 16) & 0x3FFF;
	//const int xmin = tp_calx & 0x3FFF;
	//const int ymax = (tp_caly >> 16) & 0x3FFF;
	//const int ymin = tp_caly & 0x3FFF;
	
	//const int xmax = mgos_sys_config_get_xpt2046_dac_portrait_min_xmax();
	//const int xmin = mgos_sys_config_get_xpt2046_dac_portrait_max_xmin();
	//const int ymax = mgos_sys_config_get_xpt2046_dac_landscape_min_ymax();
	//const int ymin = mgos_sys_config_get_xpt2046_dac_landscape_max_ymin();

	const int xmax = 451;
	const int xmin = 3701;
	const int ymax = 311;
	const int ymin = 3801;
	
	switch(_lcd_orientation)
	{
	case XPT2046_LANDSCAPE: // 1
	    *x_out = map(y, ymin, xmax, 0, s_max_x);
	    *y_out = map(x, xmin, ymax, 0, s_max_y);
	    break;
	case XPT2046_PORTRAIT_FLIP: // 2
	    *x_out = s_max_y - map(x, xmin, xmax, 0, s_max_y);
	    *y_out = map(y, ymin, ymax, 0, s_max_x);
	    break;
	case XPT2046_LANDSCAPE_FLIP:  // 3
	    *x_out = map(y, ymin, xmax, 0, s_max_x);
	    *y_out = s_max_y - map(x, xmin, ymax, 0, s_max_y);
	    break;
	case XPT2046_LANDSCAPE_FLIP_REVERSE:  // 4  igual que el 3 pero invirtiendo x
	    //*x_out = map(y, ymin, xmax, 0, s_max_x);  //ok = 3 (XPT2046_LANDSCAPE_FLIP)
	    *x_out = s_max_x - map(y, ymin, xmax, 0, s_max_x);
	    *y_out = s_max_y - map(x, xmin, ymax, 0, s_max_y);
	  
	    break;
	default: // XPT2046_PORTRAIT
	    *x_out = s_max_y - map(x, xmin, xmax, 0, s_max_y);
	    *y_out = map(y, ymin, ymax, 0, s_max_x);
	}
	
	LOG(LL_INFO, (" x:%d, xmin:%d, xmax:%d, *x_out:%d, s_max_x:%d", x, xmin, xmax, *x_out, s_max_x));
	LOG(LL_INFO, (" y:%d, ymin:%d, ymax:%d, *y_out:%d, s_max_y:%d", y, ymin, ymax, *y_out, s_max_y));
	
	
}


//
//
void xpt2046_read_timer_cb(void *arg)
{
	int tx, ty, tz;
	int pin = (int)arg;

	bool touch_state = xpt2046_read_touch(&tx, &ty, &tz);

	if (touch_state) {
		xpt2046_map_rotation(tx, ty, &xpt_last_touch.x, &xpt_last_touch.y);
		printf("XPT touch %d %d -> %d %d\n", tx, ty, xpt_last_touch.x, xpt_last_touch.y);

//		mgos_ili9341_set_fgcolor565(ILI9341_GREEN);
//		mgos_ili9341_drawCircle(xpt_last_touch.x, xpt_last_touch.y, 3);
		xpt_last_touch.length++;
		xpt_last_touch.z = tz;

		mgos_set_timer(1000, 0, xpt2046_read_timer_cb, (void *)pin);
	} else {
		xpt_last_touch.direction = TOUCH_UP;
		mgos_gpio_enable_int(pin);
	}
	if (xpt_event_handler) xpt_event_handler(&xpt_last_touch);
}



//
//
void xpt2046_intr_handler(const int pin, void *arg)
{
 	struct mgos_xpt2046_event_data ed;
	const bool pin_state = mgos_gpio_read(pin);
	int touch_state = 0;
	int tx=0, ty=0, tz;
  //LOG(LL_INFO, ("****xpt2046_intr_handler linea_1  at X:%d, Y:%d Z:%d", tx, ty, tz));


	if (!pin_state)
	{

  	//LOG(LL_INFO, ("****xpt2046_intr_handler linea_2  at X:%d, Y:%d Z:%d", tx, ty, tz));


		if ((touch_state = xpt2046_read_touch(&tx, &ty, &tz))) {
  			//LOG(LL_INFO, ("****xpt2046_intr_handler linea_3  at X:%d, Y:%d Z:%d", tx, ty, tz));


			// To avoid an interrupt flood from touch, set a timer and disable the interrupt
			mgos_set_timer(1000, 0, xpt2046_read_timer_cb, (void *)pin);
			mgos_gpio_disable_int(pin);

			xpt2046_map_rotation(tx, ty, &ed.x, &ed.y);

	        ed.length=1;
	        ed.direction = TOUCH_DOWN;
	        ed.z = tz;

	        memcpy((void *)&xpt_last_touch, (void *)&ed, sizeof(xpt_last_touch));
		}
		else
		{
  			//LOG(LL_INFO, ("****xpt2046_intr_handler linea_4  at X:%d, Y:%d Z:%d", tx, ty, tz));

	        ed.length=1;
	        ed.direction = TOUCH_UP;
	        ed.x = 0;
	        ed.y = 0;
	        ed.z = 0;
		}

		if (xpt_event_handler) xpt_event_handler(&ed);
	}
}


// Mongoose-OS init
//
bool mgos_xpt2046_init(void)
{
	esp_err_t ret;

	//_lcd_orientation = mgos_sys_config_get_tft_orientation();
	_lcd_orientation = mgos_sys_config_get_xpt2046_orientation();
	xpt_last_touch.direction = TOUCH_UP;  //evitar backlightkeepalive

	LOG(LL_INFO, ("XPT2046 irq pin '%d' cs index '%d'", mgos_sys_config_get_xpt2046_irq_pin(), mgos_sys_config_get_xpt2046_cs_index()));

	// ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
	struct mgos_config_spi ep_bus_cfg = {
	  .unit_no = 2,
	  .miso_gpio = mgos_sys_config_get_spi_miso_gpio(),
	  .mosi_gpio = mgos_sys_config_get_spi_mosi_gpio(),
	  .sclk_gpio = mgos_sys_config_get_spi_sclk_gpio(),
	  //.cs0_gpio = mgos_sys_config_get_xpt2046_cs_pin(),
	  .cs0_gpio = 32,
	  .debug = true,
	};
#endif

#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
	xpt2046_spi = mgos_spi_create(&ep_bus_cfg);
	xpt2046_bus_cfg = ep_bus_cfg;
#else
	xpt2046_spi = mgos_spi_get_global();
#endif

	if (xpt2046_spi == NULL) {
		LOG(LL_ERROR, ("SPI is not configured, make sure spi.enable is true"));
		return 0;
	}

	struct mgos_spi_txn txn = {
	  .cs = mgos_sys_config_get_xpt2046_cs_index(),		/* Use CS0 line as configured by cs0_gpio */
	  .mode = 0,
	  .freq = 500000,
	};
	xpt2046_txn = txn;

	gpio_pad_select_gpio(mgos_sys_config_get_xpt2046_irq_pin());
	mgos_gpio_set_mode(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_MODE_INPUT);
	mgos_gpio_set_pull(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_PULL_NONE);
	mgos_gpio_set_int_handler(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_INT_EDGE_NEG, xpt2046_intr_handler, NULL);
	mgos_gpio_enable_int(mgos_sys_config_get_xpt2046_irq_pin());

	LOG(LL_INFO, ("CS%d IRQ=%d", mgos_sys_config_get_xpt2046_cs_index(), mgos_sys_config_get_xpt2046_irq_pin() ));

	return true;
}

bool mgos_xpt2046_is_touching(void)
{
	return (xpt_last_touch.direction == TOUCH_DOWN);
}

void mgos_xpt2046_set_dimensions(const uint16_t x, const uint16_t y)
{
	s_max_x = x;  //tama?o en pixel de la pantalla en eje x
	s_max_y = y;  //tama?o en pixel de la pantalla en eje y
	LOG(LL_INFO, ("X=%d Y=%d", x, y ));
}

void mgos_xpt2046_set_handler(mgos_xpt2046_event_t handler) {
  xpt_event_handler = handler;
}


void mgos_xpt2046_set_rotation(enum mgos_xpt2046_rotation_t rotation) {
  _lcd_orientation = rotation;
}
