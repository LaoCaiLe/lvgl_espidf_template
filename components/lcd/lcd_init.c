#include "lcd_init.h"
#include <string.h>


#define LCD_HOST   VSPI_HOST
spi_device_handle_t spi;

extern void spi_disp_flush_ready(void);

void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
IRAM_ATTR void VSPI_data_queue(uint16_t *dat, uint32_t len, uint32_t user_fg)
{
    static spi_transaction_t t[80];
    static uint32_t i = 0;

    memset(&t[i], 0, sizeof(spi_transaction_t)); // Zero out the transaction
    t[i].length = len;                           // Command is 8 bits
    t[i].tx_buffer = dat;                        // The data is the cmd itself
    t[i].user = (void *)user_fg;                 // D/C needs to be set to 0
    esp_err_t ret = spi_device_queue_trans(spi, &t[i], portMAX_DELAY);
    assert(ret == ESP_OK); // Should have had no issues.
    i++;
    if (i == 80)
    {
        i = 0;
    }
}

IRAM_ATTR  void spi_ready(spi_transaction_t *trans)
{
    uint32_t spi_cnt = (uint32_t)trans->user;

    if (spi_cnt == 4)
    {
        spi_disp_flush_ready();
    }
}
void lcd_spi_init()
{
     esp_err_t ret;
    
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=LCD_MOSI_PIN,
        .sclk_io_num=LCD_SCLK_PIN,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=4094
    };
	spi_device_interface_config_t devcfg = {		
		.clock_speed_hz = SPI_MASTER_FREQ_80M, // Clock out at 10 MHz

		.mode = 0,					// SPI mode 0
		.spics_io_num = LCD_CS_PIN, // CS pin
		// .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
		.cs_ena_pretrans = 1,
		.queue_size = 80,
		.post_cb = spi_ready,//注册一个SPI调用完成的回调
	};
    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);


   
}

void LCD_GPIO_Init()
{
    gpio_reset_pin(LCD_BLK_PIN );
    gpio_reset_pin(LCD_RES_PIN );
    gpio_reset_pin(LCD_DC_PIN  );
    // gpio_reset_pin(LCD_MOSI_PIN);
    // gpio_reset_pin(LCD_SCLK_PIN);
    // gpio_reset_pin(LCD_CS_PIN  );
   
    gpio_set_direction(LCD_BLK_PIN , GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_RES_PIN , GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_DC_PIN  , GPIO_MODE_OUTPUT);
    // gpio_set_direction(LCD_MOSI_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(LCD_SCLK_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(LCD_CS_PIN  , GPIO_MODE_OUTPUT);

	// gpio_set_level(LCD_BLK_PIN , 1);
    // gpio_set_level(LCD_RES_PIN , 1);
    // gpio_set_level(LCD_DC_PIN  , 1);
    // gpio_set_level(LCD_MOSI_PIN, 1);
    // gpio_set_level(LCD_SCLK_PIN, 1);
    // gpio_set_level(LCD_CS_PIN  , 1);
}  
/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{	
// 	u8 i;
// 	LCD_CS_Clr();
// 	for(i=0;i<8;i++)
// 	{			  
// 		LCD_SCLK_Clr();
// 		if(dat&0x80)
// 		{
// 		   LCD_MOSI_Set();
// 		}
// 		else
// 		{
// 		   LCD_MOSI_Clr();
// 		}
// 		LCD_SCLK_Set();
// 		dat<<=1;
// 	}	
//   LCD_CS_Set();	

 lcd_cmd( dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_DC_Set();//写数据
	lcd_cmd( dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_DC_Set();//写数据
	lcd_cmd(dat>>8);
	lcd_cmd(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	// LCD_DC_Clr();//写命令
	// LCD_Writ_Bus(dat);
	// LCD_DC_Set();//写数据
	LCD_DC_Clr();//写命令
	lcd_cmd( dat);
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+80);
		LCD_WR_DATA(y2+80);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+80);
		LCD_WR_DATA(x2+80);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO
	lcd_spi_init();

	LCD_RES_Clr();//复位
	DELAY(10);
	LCD_RES_Set();
	DELAY(10);

	LCD_BLK_Set();//打开背光
	DELAY(10);
	LCD_WR_REG(0x11);
	DELAY(12);
	LCD_WR_REG(ST7789_SLPOUT);   // Sleep out
	vTaskDelay(120);

	LCD_WR_REG(ST7789_NORON);    // Normal display mode on

	//------------------------------display and color format setting--------------------------------//
	LCD_WR_REG(ST7789_MADCTL);
	LCD_WR_DATA8(0x08);

	// JLX240 display datasheet
	LCD_WR_REG(0xB6);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x82);

	LCD_WR_REG(ST7789_RAMCTRL);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xE0); // 5 to 6 bit conversion: r0 = r5, b0 = b5

	LCD_WR_REG(ST7789_COLMOD);
	LCD_WR_DATA8(0x55);
	vTaskDelay(10);

	//--------------------------------ST7789V Frame rate setting----------------------------------//
	LCD_WR_REG(ST7789_PORCTRL);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(ST7789_GCTRL);      // Voltages: VGH / VGL
	LCD_WR_DATA8(0x35);

	//---------------------------------ST7789V Power setting--------------------------------------//
	LCD_WR_REG(ST7789_VCOMS);
	LCD_WR_DATA8(0x28);		// JLX240 display datasheet

	LCD_WR_REG(ST7789_LCMCTRL);
	LCD_WR_DATA8(0x0C);

	LCD_WR_REG(ST7789_VDVVRHEN);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0xFF);

	LCD_WR_REG(ST7789_VRHS);       // voltage VRHS
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(ST7789_VDVSET);
	LCD_WR_DATA8(0x20);

	LCD_WR_REG(ST7789_FRCTR2);
	LCD_WR_DATA8(0x0f);

	LCD_WR_REG(ST7789_PWCTRL1);
	LCD_WR_DATA8(0xa4);
	LCD_WR_DATA8(0xa1);

	//--------------------------------ST7789V gamma setting---------------------------------------//
	LCD_WR_REG(ST7789_PVGAMCTRL);
	LCD_WR_DATA8(0xd0);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x32);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x42);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0e);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x17);

	LCD_WR_REG(ST7789_NVGAMCTRL);
	LCD_WR_DATA8(0xd0);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x31);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x47);
	LCD_WR_DATA8(0x0e);
	LCD_WR_DATA8(0x1c);
	LCD_WR_DATA8(0x17);
	LCD_WR_DATA8(0x1b);
	LCD_WR_DATA8(0x1e);

	LCD_WR_REG(ST7789_INVON);

	LCD_WR_REG(ST7789_CASET);    // Column address set
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xE5);    // 239

	LCD_WR_REG(ST7789_RASET);    // Row address set
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0xE5);    // 319

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	vTaskDelay(12);

	LCD_WR_REG(ST7789_DISPON);    //Display on
	vTaskDelay(12);
	// LCD_WR_REG(0x11); 
	// DELAY(120);

	// LCD_WR_REG(0x36);
	// LCD_WR_DATA8(0x08);
	// LCD_WR_REG(0x3A); 
	// LCD_WR_DATA8(0x05);

	// LCD_WR_REG(0xB2);
	// LCD_WR_DATA8(0x0C);
	// LCD_WR_DATA8(0x0C);
	// LCD_WR_DATA8(0x00);
	// LCD_WR_DATA8(0x33);
	// LCD_WR_DATA8(0x33); 

	// LCD_WR_REG(0xB7); 
	// LCD_WR_DATA8(0x35);  

	// LCD_WR_REG(0xBB);
	// LCD_WR_DATA8(0x19);

	// LCD_WR_REG(0xC0);
	// LCD_WR_DATA8(0x2C);

	// LCD_WR_REG(0xC2);
	// LCD_WR_DATA8(0x01);

	// LCD_WR_REG(0xC3);
	// LCD_WR_DATA8(0x12);   

	// LCD_WR_REG(0xC4);
	// LCD_WR_DATA8(0x20);  

	// LCD_WR_REG(0xC6); 
	// LCD_WR_DATA8(0x0F);    

	// LCD_WR_REG(0xD0); 
	// LCD_WR_DATA8(0xA4);
	// LCD_WR_DATA8(0xA1);

	// LCD_WR_REG(0xE0);
	// LCD_WR_DATA8(0xD0);
	// LCD_WR_DATA8(0x04);
	// LCD_WR_DATA8(0x0D);
	// LCD_WR_DATA8(0x11);
	// LCD_WR_DATA8(0x13);
	// LCD_WR_DATA8(0x2B);
	// LCD_WR_DATA8(0x3F);
	// LCD_WR_DATA8(0x54);
	// LCD_WR_DATA8(0x4C);
	// LCD_WR_DATA8(0x18);
	// LCD_WR_DATA8(0x0D);
	// LCD_WR_DATA8(0x0B);
	// LCD_WR_DATA8(0x1F);
	// LCD_WR_DATA8(0x23);

	// LCD_WR_REG(0xE1);
	// LCD_WR_DATA8(0xD0);
	// LCD_WR_DATA8(0x04);
	// LCD_WR_DATA8(0x0C);
	// LCD_WR_DATA8(0x11);
	// LCD_WR_DATA8(0x13);
	// LCD_WR_DATA8(0x2C);
	// LCD_WR_DATA8(0x3F);
	// LCD_WR_DATA8(0x44);
	// LCD_WR_DATA8(0x51);
	// LCD_WR_DATA8(0x2F);
	// LCD_WR_DATA8(0x1F);
	// LCD_WR_DATA8(0x1F);
	// LCD_WR_DATA8(0x20);
	// LCD_WR_DATA8(0x23);

	// LCD_WR_REG(0x21); 

	// LCD_WR_REG(0x11);

	// LCD_WR_REG(0x29);
} 










