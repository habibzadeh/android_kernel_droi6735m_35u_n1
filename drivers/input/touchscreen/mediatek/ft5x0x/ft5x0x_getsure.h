#define FTS_GESTRUE

#include "accdet.h"

#ifdef FTS_GESTRUE
#define GESTURE_LEFT								0x20
#define GESTURE_RIGHT								0x21
#define GESTURE_UP		    							0x22
#define GESTURE_DOWN								0x23
#define GESTURE_DOUBLECLICK						0x24
#define GESTURE_O		    							0x30
#define GESTURE_W		    							0x31
#define GESTURE_M		   	 						0x32
#define GESTURE_E		    							0x33
#define GESTURE_L		    							0x44
#define GESTURE_S		    							0x46
#define GESTURE_V		    							0x54
#define GESTURE_Z		    							0x41

#define KEY_GESTURE_U 						KEY_U
#define KEY_GESTURE_UP 						KEY_UP
#define KEY_GESTURE_DOWN 					KEY_DOWN
#define KEY_GESTURE_LEFT 					KEY_LEFT 
#define KEY_GESTURE_RIGHT 					KEY_RIGHT
#define KEY_GESTURE_O 						KEY_O
#define KEY_GESTURE_E 						KEY_E
#define KEY_GESTURE_M 						KEY_M 
#define KEY_GESTURE_L 						KEY_L
#define KEY_GESTURE_W 						KEY_W
#define KEY_GESTURE_S 						KEY_S 
#define KEY_GESTURE_V 						KEY_V
#define KEY_GESTURE_Z 						KEY_Z


#define FTS_DMA_BUF_SIZE	1024


#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif

static char tpgesture_value[10]={};
static char tpgesture_status_value[5] = {};
static char tpgesture_status	= 1;
//static int g_call_state 	= 0;

extern struct i2c_client *i2c_client;
extern struct tpd_device *tpd;

int fts_Gesture_init(struct input_dev *input_dev)
{
        //init_para(480,854,60,0,0);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
		
	__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	__set_bit(KEY_GESTURE_UP, input_dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	__set_bit(KEY_GESTURE_U, input_dev->keybit);
	__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);

	return 0;
}

static void fts_check_gesture(struct input_dev *input_dev,int gesture_id)
{
	printk("fts gesture_id==0x%x\n ",gesture_id);

	switch(gesture_id)
	{
	        case GESTURE_LEFT:
	                input_report_key(input_dev, KEY_GESTURE_LEFT, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_LEFT, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_RIGHT:
	                input_report_key(input_dev, KEY_GESTURE_RIGHT, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_RIGHT, 0);
	                input_sync(input_dev);
			break;
	        case GESTURE_UP:
	                input_report_key(input_dev, KEY_GESTURE_UP, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_UP, 0);
	                input_sync(input_dev);                                  
	                break;
	        case GESTURE_DOWN:
	                input_report_key(input_dev, KEY_GESTURE_DOWN, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_DOWN, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_DOUBLECLICK:
	                input_report_key(input_dev, KEY_GESTURE_U, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_U, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_O:
	                input_report_key(input_dev, KEY_GESTURE_O, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_O, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_W:
	                input_report_key(input_dev, KEY_GESTURE_W, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_W, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_M:
	                input_report_key(input_dev, KEY_GESTURE_M, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_M, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_E:
	                input_report_key(input_dev, KEY_GESTURE_E, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_E, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_L:
	                input_report_key(input_dev, KEY_GESTURE_L, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_L, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_S:
	                input_report_key(input_dev, KEY_GESTURE_S, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_S, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_V:
	                input_report_key(input_dev, KEY_GESTURE_V, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_V, 0);
	                input_sync(input_dev);
	                break;
	        case GESTURE_Z:
	                input_report_key(input_dev, KEY_GESTURE_Z, 1);
	                input_sync(input_dev);
	                input_report_key(input_dev, KEY_GESTURE_Z, 0);
	                input_sync(input_dev);
	                break;
	        default:
	                break;
	}
}

static int ft5x0x_read_Touchdata(struct i2c_client* i2c_client)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
	int ret = -1;
	int i = 0;

	int gestrue_id = 0;
	short pointnum = 0;

	buf[0] = 0xd3;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	printk("%s lsm--buf[0]=%x .\n",__func__,buf[0]);
	if(buf[0] != 0xfe)
	{
// pass dblclick
		gestrue_id =  buf[0];
		fts_check_gesture(tpd->dev, gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;
	printk("%s lsm--pointnum=%d .\n",__func__, pointnum);

	buf[0] = 0xd3;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, pointnum * 4 + 2+6);
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

//	gestrue_id = fetch_object_sample(buf, pointnum);
//	fts_check_gesture(tpd->dev, gestrue_id);
//	printk( "%d read gestrue_id.\n", gestrue_id);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		printk( "Gesture touch pint x,y=%d,%d\n", coordinate_x[i], coordinate_y[i]);
	}
	return -1;
}

 static int touch_getsure_event_handler(struct i2c_client* i2c_client)
{
 	u8 state = 0;
//	i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
	fts_read_reg(i2c_client, 0xd0, &state);
	printk("%s lsm--state=%x .\n",__func__,state);
	if(state !=1) return false;

	ft5x0x_read_Touchdata(i2c_client);
	return true;
}

static bool tpd_getsure_suspend(struct i2c_client* i2c_client)
 {
	printk("[xy-tp]%d\n", tpgesture_status);

//	if((g_call_state == CALL_ACTIVE) || (!tpgesture_status)) 
//		return false;

	printk("[xy-tp]:gesture mode\n");
//	msleep(200);

	fts_write_reg(i2c_client, 0xd0, 0x01);
	fts_write_reg(i2c_client, 0xd1, 0xff);
	fts_write_reg(i2c_client, 0xd2, 0xff);
	fts_write_reg(i2c_client, 0xd5, 0xff);
	fts_write_reg(i2c_client, 0xd6, 0xff);
	fts_write_reg(i2c_client, 0xd7, 0xff);
	fts_write_reg(i2c_client, 0xd8, 0xff);

	return true;
 }

static bool tpd_getsure_resume(struct i2c_client* i2c_client)
 {
	TPD_DMESG("TPD wake up\n");
//	if((g_call_state == CALL_ACTIVE) || (!tpgesture_status))
//		return false;

	fts_write_reg(i2c_client, 0xD0, 0x0);

	return true;
 }

static ssize_t show_tpgesture_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	 printk("show tp gesture value is %s \n", tpgesture_value);
	 return sprintf(buf, "%s\n", tpgesture_value);
}
 
static ssize_t show_tpgesture_status_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	 printk("show tp gesture status is %s\n", tpgesture_status_value);
	 return sprintf(buf, "%s\n", tpgesture_status_value);
}
 
static ssize_t store_tpgesture_status_value(struct device* dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	 if(!strncmp(buf, "on", 2))
	 {
		 sprintf(tpgesture_status_value,"on");
		 tpgesture_status = 1;//status --- on
	 }
	 else
	 {
		 sprintf(tpgesture_status_value,"off");
		 tpgesture_status = 0;//status --- off
	 }
 
	 return count;
}

static DEVICE_ATTR(tpgesture, 0664, show_tpgesture_value, NULL);
//	0666
static DEVICE_ATTR(tpgesture_status, 0664, show_tpgesture_status_value, store_tpgesture_status_value);


