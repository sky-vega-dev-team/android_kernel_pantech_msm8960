/* 
 *  ti_drv2665.c - Texas Instruments DRV2665 piezo haptic driver 
 * 
 *  Copyright (C) 2011 Texas Instruments 
 * 
 *  Contact: Ameya Palande <ameya.palande@ti.com> 
 * 
 *  This program is free software; you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation; either version 2 of the License, or 
 *  (at your option) any later version. 
 * 
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details. 
 * 
 *  You should have received a copy of the GNU General Public License 
 *  along with this program; if not, write to the Free Software 
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
 */  
  
#include <linux/module.h>  
#include <linux/slab.h>  
#include <linux/i2c.h>  
#include <linux/pm.h>  
#include <linux/delay.h>  
#include <linux/regulator/consumer.h>  
#include <linux/gpio.h>  
#include <linux/fs.h>  
  
#include "../staging/android/timed_output.h"  
  
#ifdef CONFIG_HAS_EARLYSUSPEND  
#include <linux/earlysuspend.h>  
#define DRV2665_SUS_LEVEL   1  
#endif  
  
//#ifdef CONFIG_PIEZO_ECO  
#define DEVICE_BUS 5  
//#else  
//#define DEVICE_BUS  19  
//#endif  
  
#if 0  
/* 
** I2C FW info 
*/  
typedef struct _FWINFO  
{  
    char cMajorVersion;  
    char cMinorVersion;  
    char cInterfaceType;    /* 3 = I2C */  
    char cAmpType[9];       /* 1 = differential amp */  
    char cAmpUsage;         /* 1 = single boost, 2 = dual boost */  
    char cClockSource;      /* 2 = external 18MHz clock */  
} FWINFO;  
FWINFO g_FWInfo;  
  
#define VIBE_MAX_DEVICE_NAME_LENGTH         64  
static char g_szFWVersion[VIBE_MAX_DEVICE_NAME_LENGTH];  
#endif  
  
#define DRV2665_DRV_NAME            "ti_drv2665"  
#define DRV2665_I2C_ADDRESS         0x59  
#define DRV2665_CHIP_ID             0x05  
#define DRV2665_AUTOSUSPEND_DELAY       5000  
#define DRV2665_FIFO_DEPTH          100  
#define DRV2665_MSG_BUFFER_SIZE         PAGE_SIZE  
  
#define DRV2665_STATUS_REG          0x00  
#define DRV2665_STATUS_MASK     0x03  
#define DRV2665_CONTROL_REG         0x01  
#define DRV2665_ID_MASK         0x78  
#define DRV2665_ID_SHIFT        0x03  
#define DRV2665_CONTROL_WRITE_MASK  0x07  
#define DRV2665_CONTROL2_REG            0x02  
#define DRV2665_DEV_RST_SHIFT       0x07  
#define DRV2665_DEV_RST         (1 << DRV2665_DEV_RST_SHIFT)  
#define DRV2665_STANDBY_SHIFT       0x06  
#define DRV2665_STANDBY         (1 << DRV2665_STANDBY_SHIFT)  
#define DRV2665_TIMEOUT_SHIFT       0x02  
#define DRV2665_05MS_TIMEOUT        0x00  
#define DRV2665_10MS_TIMEOUT        0x01  
#define DRV2665_15MS_TIMEOUT        0x02  
#define DRV2665_20MS_TIMEOUT        0x03  
#define DRV2665_FIFO_REG            0x0B  
  
  
/* 
Following  code is for setting gain for DRV2665  in register 1 bit 0 and bit 1 
0x00 :output  is set for +/-25V :For AAc, this is more useful  for +/-30V actuator 
0x01 :output  is set for +/-50V 
0x02 :output  is set for +/-75V :we use this for SEMCO 
0x03 :output  is set for +/-100V 
*/  
#define DRV2665_GAIN_50 0x00  
#define DRV2665_GAIN_100 0x01  
#define DRV2665_GAIN_150 0x02  
#define DRV2665_GAIN_200 0x03  
  
struct drv2665_data {  
  
    //--------Timer ---------//  
    struct i2c_client *client;  
    struct timed_output_dev timed_dev;  
    struct hrtimer timer;  
    struct work_struct work;  
    u32 max_runtime_ms;  
    u32 time_chunk_ms;  
    u32 runtime_left;     
#ifdef CONFIG_HAS_EARLYSUSPEND  
    struct early_suspend es;  
#endif  
    //----------------------//  
      
    struct mutex lock;  
    /* DRV2665 cached registers */  
    u8 control;  
    u8 control2;  
    /* DRV2665_FIFO_DEPTH + additional 1 byte for fifo cmd register */  
    u8 fifo_buff[DRV2665_FIFO_DEPTH + 1];  
      
  
};  
  
static int drv2665_read_byte(struct device *dev, u8 reg, const char *reg_name)  
{  
    int val;  
  
    val = i2c_smbus_read_byte_data(to_i2c_client(dev), reg);  
    if (val < 0)  
        dev_err(dev, "error reading %s register\n", reg_name);  
  
    return val;  
}  
  
static int drv2665_write_byte(struct device *dev, u8 reg, u8 data,  
        const char *reg_name)  
{  
    int status;  
  
    status = i2c_smbus_write_byte_data(to_i2c_client(dev), reg, data);  
    if (status < 0)  
        dev_err(dev, "error writing %s register\n", reg_name);  
  
    return status;  
}  
  
static ssize_t status_show(struct device *dev, struct device_attribute *attr,  
        char *buf)  
{  
    int val;  
  
    val = drv2665_read_byte(dev, DRV2665_STATUS_REG, "status");  
    if (val < 0)  
        return val;  
  
    return sprintf(buf, "%x\n", val & DRV2665_STATUS_MASK);  
}  
static DEVICE_ATTR(status, S_IRUGO, status_show, NULL);  
  
static ssize_t fifo_store(struct device *dev, struct device_attribute *attr,  
        const char *buf, size_t count)  
{  
    int status;  
    struct i2c_client *client;  
    struct drv2665_data *data;  
    struct i2c_msg msg;  
  
    /* 
     * make sure that count is between 1 and DRV2665_FIFO_DEPTH both 
     * inclusive 
     */  
    if (count < 1)  
        return -EINVAL;  
    if (count > DRV2665_FIFO_DEPTH)  
        return -ENOSPC;  
  
    memset(&msg, 0x00, sizeof(msg));  
    client = to_i2c_client(dev);  
    data = i2c_get_clientdata(client);  
  
    mutex_lock(&data->lock);  
  
    /* DRV2665 FIFO Register Address */  
    data->fifo_buff[0] = DRV2665_FIFO_REG;  
  
    /* fill remaining fifo_buff with data from user space */  
    memcpy(data->fifo_buff + 1, buf, count);  
    msg.addr = client->addr;  
    msg.flags = 0;  
    msg.buf = data->fifo_buff;  
    msg.len = count + 1;  
    //msg.transferred = 0;  
  
    status = i2c_transfer(client->adapter, &msg, 1);  
  
    mutex_unlock(&data->lock);  
  
    /* 
     * subtract DRV2665_FIFO_REG byte from successfully 
     * transferred bytes 
     */  
    //return msg.transferred - 1;  
    return count;  
}  
static ssize_t fifo_show(struct device *dev,  
        struct device_attribute *attr, char *buf)  
{  
    struct i2c_client *client;  
    struct drv2665_data *data;  
    int i, count = 0, count_total = 0;  
    char * buf_tmp;  
  
    client = to_i2c_client(dev);  
    data = i2c_get_clientdata(client);  
  
    mutex_lock(&data->lock);  
      
    buf_tmp = buf;  
    for(i=0;i<DRV2665_FIFO_DEPTH;i++)  
    {  
        count = sprintf(buf_tmp, "%d\n", data->fifo_buff[i]);  
        buf_tmp += count;  
        count_total += count;  
    }  
      
    mutex_unlock(&data->lock);  
  
    return count_total;  
}  
  
static DEVICE_ATTR(fifo, S_IRUGO | S_IWUSR, fifo_show, fifo_store);  
  
static ssize_t control_show(struct device *dev,  
        struct device_attribute *attr, char *buf)  
{  
    int val;  
  
    val = drv2665_read_byte(dev, DRV2665_CONTROL_REG, "control");  
    if (val < 0)  
        return val;  
  
    return sprintf(buf, "%x\n", val);  
}  
  
static ssize_t control_store(struct device *dev,  
        struct device_attribute *attr, const char *buf, size_t count)  
{  
    u8 val;  
    int status;  
    struct drv2665_data *data;  
    ssize_t ret_value;  
  
    data = dev_get_drvdata(dev);  
  
    status = kstrtou8(buf, 16, &val);  
    if (status)  
        return status;  
  
    mutex_lock(&data->lock);  
  
    status = drv2665_write_byte(dev, DRV2665_CONTROL_REG,  
            val & DRV2665_CONTROL_WRITE_MASK, "control");  
    if (status < 0) {  
        ret_value = status;  
    } else {  
        /* cache DRV2665_CONTROL_REG value */  
        data->control = val & DRV2665_CONTROL_WRITE_MASK;  
        ret_value = 1;  
    }  
  
    mutex_unlock(&data->lock);  
  
    return ret_value;  
}  
static DEVICE_ATTR(control, S_IRUGO | S_IWUSR, control_show, control_store);  
  
static ssize_t control2_show(struct device *dev,  
        struct device_attribute *attr, char *buf)  
{  
    int val;  
  
    val = drv2665_read_byte(dev, DRV2665_CONTROL2_REG, "control2");  
    if (val < 0)  
        return val;  
  
    return sprintf(buf, "%x\n", val);  
}  
  
static ssize_t control2_store(struct device *dev,  
        struct device_attribute *attr, const char *buf, size_t count)  
{  
    u8 val;  
    int status;  
    struct drv2665_data *data;  
    ssize_t ret_value;  
  
    data = dev_get_drvdata(dev);  
  
    status = kstrtou8(buf, 16, &val);  
    if (status)  
        return status;  
  
    mutex_lock(&data->lock);  
  
    status = drv2665_write_byte(dev, DRV2665_CONTROL2_REG, val, "control2");  
    if (status < 0) {  
        ret_value = status;  
    } else {  
        /* cache DRV2665_CONTROL2_REG value */  
        data->control2 = val;  
        ret_value = 1;  
    }  
  
    mutex_unlock(&data->lock);  
  
    return ret_value;  
}  
static DEVICE_ATTR(control2, S_IRUGO | S_IWUSR, control2_show, control2_store);  
  
static ssize_t reset_store(struct device *dev,  
        struct device_attribute *attr, const char *buf, size_t count)  
{  
    u8 val;  
    int status;  
    struct drv2665_data *data;  
  
    data = dev_get_drvdata(dev);  
  
    status = kstrtou8(buf, 16, &val);  
    if (status)  
        return status;  
  
    if (val != 1)  
        return -EINVAL;  
  
    mutex_lock(&data->lock);  
    status = drv2665_write_byte(dev, DRV2665_CONTROL2_REG,  
            DRV2665_DEV_RST, "control2");  
    mutex_unlock(&data->lock);  
  
    if (status < 0)  
        return status;  
  
    return 1;  
}  
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset_store);  
  
  
static struct attribute *drv2665_attributes[] = {  
    &dev_attr_reset.attr,  
    &dev_attr_control2.attr,  
    &dev_attr_control.attr,  
    &dev_attr_status.attr,  
    &dev_attr_fifo.attr,  
    NULL  
};  
  
static const struct attribute_group drv2665_attr_group = {  
    .attrs = drv2665_attributes,  
};  
  
  
//-------------------------------------------Timer ------------------------------------------//  
#define DRV2665_BYTES_PER_MS    8 //8K HZ  
#define DRV2665_VIB_DEFAULT_VAL 0x7F  
#define DRV2665_VIB_MAX_TIME    5000  
#define DRV2665_FIFO_CHUNK_MS    5
  
#define DRV2665_GAIN DRV2665_GAIN_150  
#define DRV2665_TIMEOUT (DRV2665_20MS_TIMEOUT << DRV2665_TIMEOUT_SHIFT)  
  
u8 sin_buf[]={0,7,15,23,30,38,45,53,60,68,75,82,90,97,104,111,118,124,131,138,144,150,156,162,168,174,180,185,190,195,200,205,209,214,218,222,225,229,232,235,238,241,243,245,247,249,251,252,253,254,255,255,255,255,255,255,254,253,252,250,249,247,245,243,240,237,234,231,228,224,220,217,212,208,203,199,194,189,183,178,172,167,161,155,149,142,136,129,123,116,109,102,95,88,80,73,66,58,51,43,36,28,20,13,5,};  
  
static int drv2665_read_reg(struct i2c_client *client, u32 reg)  
{  
    int rc;  
  
    rc = i2c_smbus_read_byte_data(client, reg);  
    if (rc < 0)  
        dev_err(&client->dev, "i2c reg read for 0x%x failed\n", reg);  
    return rc;  
}  
  
static int drv2665_write_reg(struct i2c_client *client, u32 reg, u8 val)  
{  
    int rc;  
  
    rc = i2c_smbus_write_byte_data(client, reg, val);  
    if (rc < 0)  
        dev_err(&client->dev, "i2c reg write for 0x%xfailed\n", reg);  
  
    return rc;  
}  
  
static void drv2665_dump_regs(struct drv2665_data *data, char *label)  
{  
    dev_dbg(&data->client->dev,  
        "%s: reg0x00 = 0x%x, reg0x01 = 0x%x reg0x02 = 0x%x", label,  
        drv2665_read_reg(data->client, DRV2665_STATUS_REG),  
        drv2665_read_reg(data->client, DRV2665_CONTROL_REG),  
        drv2665_read_reg(data->client, DRV2665_CONTROL2_REG));  
}  
  
static void drv2665_worker(struct work_struct *work)  
{  
    struct drv2665_data *data;  
    int rc = 0;  
    u8 val;  
  
    data = container_of(work, struct drv2665_data, work);  
  
    /* data is played at 8khz */  
    if (data->runtime_left < data->time_chunk_ms)  
        val = data->runtime_left * DRV2665_BYTES_PER_MS;  
    else  
        val = data->time_chunk_ms * DRV2665_BYTES_PER_MS;  
  
    rc = i2c_master_send(data->client, data->fifo_buff, val + 1);  
  
    if (rc < 0)  
        dev_err(&data->client->dev, "i2c send message failed\n");  
}  
  
static void drv2665_enable(struct timed_output_dev *dev, int runtime)  
{  
    struct drv2665_data *data = container_of(dev, struct drv2665_data, timed_dev);  
    unsigned long time_ms;  
  
    if (runtime > data->max_runtime_ms) {  
        dev_dbg(&data->client->dev, "Invalid runtime\n");  
        runtime = data->max_runtime_ms;  
    }  
  
    mutex_lock(&data->lock);  
    hrtimer_cancel(&data->timer);  
    data->runtime_left = runtime;  
    if (data->runtime_left < data->time_chunk_ms)  
        time_ms = runtime * NSEC_PER_MSEC;  
    else  
        time_ms = data->time_chunk_ms * NSEC_PER_MSEC;  
    hrtimer_start(&data->timer, ktime_set(0, time_ms), HRTIMER_MODE_REL);  
    schedule_work(&data->work);  
    mutex_unlock(&data->lock);  
}  
  
static int drv2665_get_time(struct timed_output_dev *dev)  
{  
    struct drv2665_data *data = container_of(dev, struct drv2665_data, timed_dev);  
  
    if (hrtimer_active(&data->timer))  
        return  data->runtime_left +  
            ktime_to_ms(hrtimer_get_remaining(&data->timer));  
    return 0;  
}  
  
static enum hrtimer_restart drv2665_timer(struct hrtimer *timer)  
{  
    struct drv2665_data *data;  
    int time_ms;  
  
    data = container_of(timer, struct drv2665_data, timer);  
    if (data->runtime_left <= data->time_chunk_ms) {  
        data->runtime_left = 0;  
        schedule_work(&data->work);  
        return HRTIMER_NORESTART;  
    }  
  
    data->runtime_left -= data->time_chunk_ms;  
    if (data->runtime_left < data->time_chunk_ms)  
        time_ms = data->runtime_left * NSEC_PER_MSEC;  
    else  
        time_ms = data->time_chunk_ms * NSEC_PER_MSEC;  
  
    hrtimer_forward_now(&data->timer, ktime_set(0, time_ms));  
    schedule_work(&data->work);  
    return HRTIMER_RESTART;  
}  
  
static int drv2665_pm(bool enable)  
{  
    int nRet;  
    struct regulator *vreg_lvs4_1p8;  
  
    vreg_lvs4_1p8 = regulator_get(NULL, "8921_lvs4");  
    if(IS_ERR(vreg_lvs4_1p8)) {  
        nRet = PTR_ERR(vreg_lvs4_1p8);  
        return -EIO;  
    }  
    if(enable)  
        nRet = regulator_enable(vreg_lvs4_1p8);  
    else  
        nRet = regulator_disable(vreg_lvs4_1p8);  
    msleep(50);  
    if(nRet<0) {  
        return -EIO;  
    }  
    regulator_put(vreg_lvs4_1p8);  
  
    return nRet;  
}  
  
#ifdef CONFIG_PM  
static int drv2665_suspend(struct device *dev)  
{  
    struct drv2665_data *data = dev_get_drvdata(dev);  
    u8 val;  
    int rc;  
  
    hrtimer_cancel(&data->timer);  
    cancel_work_sync(&data->work);  
  
    /* set standby */  
    val = data->control2 | DRV2665_STANDBY;  
    rc = drv2665_write_reg(data->client, DRV2665_CONTROL2_REG, val);  
    if (rc < 0)  
        dev_err(dev, "unable to set standby\n");  
  
    /* turn regulators off */  
    drv2665_pm(false);  
    return 0;  
}  
  
static int drv2665_resume(struct device *dev)  
{  
    struct drv2665_data *data = dev_get_drvdata(dev);  
    int rc = drv2665_write_reg(data->client,  
            DRV2665_CONTROL2_REG, data->control2);  
  
    /* turn regulators on */  
    drv2665_pm(true);  
    if (rc < 0) {  
        dev_err(dev, "unable to turn regulators on\n");  
        return rc;  
    }  
  
    /* clear standby */  
    if (rc < 0) {  
        dev_err(dev, "unable to clear standby\n");  
        goto vreg_off;  
    }  
  
    return 0;  
vreg_off:  
    drv2665_pm(false);  
    return rc;  
}  
  
#ifdef CONFIG_HAS_EARLYSUSPEND  
static void drv2665_early_suspend(struct early_suspend *es)  
{  
    struct drv2665_data *data = container_of(es, struct drv2665_data, es);  
  
    drv2665_suspend(&data->client->dev);  
}  
  
static void drv2665_late_resume(struct early_suspend *es)  
{  
    struct drv2665_data *data = container_of(es, struct drv2665_data, es);  
  
    drv2665_resume(&data->client->dev);  
}  
#endif  
  
static const struct dev_pm_ops drv2665_pm_ops = {  
#ifndef CONFIG_HAS_EARLYSUSPEND  
    .suspend = drv2665_suspend,  
    .resume = drv2665_resume,  
#endif  
};  
#endif  
  
//-----------------------------------------Timer end-----------------------------------------//  
  
static int __devinit drv2665_probe(struct i2c_client *client,  
        const struct i2c_device_id *id)  
{  
    int status;  
    struct drv2665_data *data;  
    int rc;  
//  char buf[1] = {0};  
      
    drv2665_pm(true);  
  
    data = kzalloc(sizeof(*data), GFP_KERNEL);  
    if (!data) {  
        status = -ENOMEM;  
        dev_err(&client->dev, "%s: kzalloc failed\n", __func__);  
        goto exit;  
    }  
  
    /* register sysfs hooks */  
    status = sysfs_create_group(&client->dev.kobj, &drv2665_attr_group);  
    if (status) {  
        dev_err(&client->dev, "sysfs_create_group failed\n");  
        goto exit_free;  
    }  
  
    /* 
     * turn on the chip with max timeout of 20ms 
     * save this configuration in dev->control2 
     */  
    data->control2 = DRV2665_20MS_TIMEOUT << DRV2665_TIMEOUT_SHIFT;  
    status = drv2665_write_byte(&client->dev, DRV2665_CONTROL2_REG,  
            data->control2, "control2");  
    if (status < 0)  
        goto exit_sysfs;  
  
    status = drv2665_write_byte(&client->dev, DRV2665_CONTROL_REG,  
            DRV2665_GAIN, "control");  
    if (status < 0)  
        goto exit_sysfs;  
  
    mutex_init(&data->lock);  
    i2c_set_clientdata(client, data);  
  
#if 0     
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))  
    {       
        dev_err(&client->dev, "i2c_check_functionality failed.\n");  
        return -ENODEV;  
    }  
    else  
    {  
        dev_info(&client->dev, "addr=0x%02x name=%s\n",client->addr, client->name);  
  
        /* Retrieve the FW version for information purposes */  
        dev_info(&client->dev, "send to master, buf=%s\n",buf);  
        i2c_master_send(client, (char*)(&buf), 1);  
        dev_info(&client->dev, "recv from master, buf=%s\n",buf);  
        if ((!client) || (i2c_master_recv(client, (char*)(&g_FWInfo), sizeof(FWINFO)) != sizeof(FWINFO)))  
        {  
            /* Could not retrieve the FW info. Writing ? as an indication */  
            dev_err(&client->dev, "Could not retrieve the FW info. Writing ? as an indication\n");  
            sprintf(g_szFWVersion, "[FW: ?]");  
        }  
        else  
        {  
            sprintf(g_szFWVersion, "[FW: v%d.%d]", g_FWInfo.cMajorVersion, g_FWInfo.cMinorVersion);  
        }  
    }  
    dev_info(&client->dev, "drv2605_probe end. %s\n",g_szFWVersion);  
#endif  
      
    //------------------Timer ------------//  
    data->client = client;  
    data->max_runtime_ms = DRV2665_VIB_MAX_TIME;  
    INIT_WORK(&data->work, drv2665_worker);  
    hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);  
    data->timer.function = drv2665_timer;  
      
    data->fifo_buff[0] = DRV2665_FIFO_REG;  
  
    memcpy(data->fifo_buff+1, sin_buf, DRV2665_FIFO_DEPTH-1);  
          
    data->time_chunk_ms = DRV2665_FIFO_CHUNK_MS;  
      
    drv2665_dump_regs(data, "new");  
      
    /* register with timed output class */  
    data->timed_dev.name = "vibrator";  
    data->timed_dev.get_time = drv2665_get_time;  
    data->timed_dev.enable = drv2665_enable;  
      
    rc = timed_output_dev_register(&data->timed_dev);  
    if (rc) {  
        dev_err(&client->dev, "unable to register with timed_output\n");  
        goto exit_sysfs;  
    }  
      
#ifdef CONFIG_HAS_EARLYSUSPEND  
    data->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + DRV2665_SUS_LEVEL;  
    data->es.suspend = drv2665_early_suspend;  
    data->es.resume = drv2665_late_resume;  
    register_early_suspend(&data->es);  
#endif  
      
    dev_info(&client->dev, "initialized successfully\n");  
  
    return 0;  
  
exit_sysfs:  
    sysfs_remove_group(&client->dev.kobj, &drv2665_attr_group);  
exit_free:  
    kfree(data);  
exit:  
    return status;  
}  
  
static int __devexit drv2665_remove(struct i2c_client *client)  
{  
    struct drv2665_data *data;  
  
    data = i2c_get_clientdata(client);  
  
    /* unregister sysfs hooks */  
    sysfs_remove_group(&client->dev.kobj, &drv2665_attr_group);  
      
#ifdef CONFIG_HAS_EARLYSUSPEND  
    unregister_early_suspend(&data->es);  
#endif  
  
    mutex_destroy(&data->lock);  
    timed_output_dev_unregister(&data->timed_dev);  
    hrtimer_cancel(&data->timer);  
    cancel_work_sync(&data->work);  
    kfree(data);  
    return 0;  
}  
  
#if 0  
static int drv2665_detect(struct i2c_client *client,  
        struct i2c_board_info *info)  
{  
    int val;  
  
    if (!i2c_check_functionality(client->adapter,  
                I2C_FUNC_SMBUS_BYTE_DATA))  
        return -ENODEV;  
  
    val = drv2665_read_byte(&client->dev, DRV2665_CONTROL_REG, "control");  
    if (val < 0)  
    {  
        dev_info(&client->dev, "control error\n");  
        return -ENODEV;  
    }  
  
    val = (val & DRV2665_ID_MASK) >> DRV2665_ID_SHIFT;  
  
    if (val != DRV2665_CHIP_ID)  
    {  
        dev_info(&client->dev, "DRV2665_CHIP_ID error\n");  
        return -ENODEV;  
    }  
    else  
        strlcpy(info->type, DRV2665_DRV_NAME, I2C_NAME_SIZE);  
  
    return 0;  
}  
#endif  
  
static const struct i2c_device_id drv2665_id[] = {  
    { DRV2665_DRV_NAME, 0 },  
    { }  
};  
MODULE_DEVICE_TABLE(i2c, drv2665_id);  
  
static const unsigned short normal_i2c[] = { DRV2665_I2C_ADDRESS,  
                        I2C_CLIENT_END };  
  
static struct i2c_driver drv2665_driver = {  
    .driver = {  
        .name   = DRV2665_DRV_NAME,  
        .owner  = THIS_MODULE,  
#ifdef CONFIG_PM  
        .pm = &drv2665_pm_ops,  
#endif  
    },  
    .id_table   = drv2665_id,  
    .probe      = drv2665_probe,  
//  .detect     = drv2665_detect,  
    .address_list   = normal_i2c,  
    .remove     = __devexit_p(drv2665_remove),  
};  
  
  
static struct i2c_board_info info = {  
  I2C_BOARD_INFO(DRV2665_DRV_NAME, DRV2665_I2C_ADDRESS),  
};  
  
static int __init drv2665_init(void)  
{  
    struct i2c_adapter* adapter; /* Initialized below. */  
    struct i2c_client* client; /* Initialized below. */  
      
    adapter = i2c_get_adapter(DEVICE_BUS);  
    if (adapter) {  
        pr_info("found device bus %d\n",DEVICE_BUS);  
        client = i2c_new_device(adapter, &info);  
        if (client) {  
            int retVal = i2c_add_driver(&drv2665_driver);  
            if (retVal) {  
                pr_err("drv2665_driver: Cannot add driver.\n");  
                return -ENODEV;  
            }  
            pr_err("added driver. retVal=%d addr=0x%02x name=%s\n",retVal, client->addr, client->name);  
            return retVal;  
        } else {  
            pr_err("drv2665_driver: Cannot create new device.\n");  
            return -ENODEV;  
        }  
  
    } else {  
        pr_err("drv2665_driver: I2C Adapter not found.\n");  
        return -ENODEV;  
    }  
      
    return -ENODEV;  
}  
  
static void __exit drv2665_exit(void)  
{  
    i2c_del_driver(&drv2665_driver);  
}  
  
module_init(drv2665_init);  
module_exit(drv2665_exit);  
  
MODULE_AUTHOR("Le Hoang <Vegasoft.vn>");  
MODULE_DESCRIPTION("DRV2665 Piezo Haptic Driver");  
MODULE_LICENSE("GPL v2");  

