#define DRIVER_NAME     "applespi:keyboard"
#define SPI_MISO_GPIO   85
#define SPI_MOSI_GPIO   86
#define SPI_SCK_GPIO    84
#define SPI_N_CHIPSEL   4
#include "spi-gpio.c"

#define SPI_HID_STARTED         0
#define SPI_HID_RESET_PENDING   1
#define SPI_HID_READ_PENDING    2

/* hid descriptor for apple keyboard */
#define SPI_BCD_VERSION_APPLE (3)
#define SPI_VENDOR_ID_APPLE (0x05ac)
#define SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO (0x0276)

static unsigned int gpioKeyboardCS_L = 83;
static unsigned int gpioKeyboardCLK = 84;
static unsigned int gpioKeyboardMISO = 85;
static unsigned int gpioKeyboardMOSI = 86;

static unsigned int gpioTrackpadCS_L = 87;
static unsigned int gpioTrackpadCLK = 88;
static unsigned int gpioTrackpadMISO = 89;
static unsigned int gpioTrackpadMOSI = 90;

#define APPLE_HID_KBD_REPORT_LENGTH (8)
#define APPLE_HID_KBD_DESC_LENGTH (208)

#define APPLE_HID_TP_REPORT_LENGTH (8)
#define APPLE_HID_TP_DESC_LENGTH (110)

struct hid_func_descriptor
{
    unsigned char subclass;
    unsigned char protocol;
    unsigned short report_length;
    unsigned short report_desc_length;
    unsigned char report_desc[];
};

struct apple_hid_data
{
  unsigned long flags; /* device flags */

  int irq;
  
  struct hid_device *hid;

  unsigned short bufsize;
  unsigned char *inbuf;         /* Input buffer */
  unsigned char *rawbuf;        /* Raw Input buffer */
  unsigned char *cmdbuf;        /* Command buffer */
  unsigned char *argsbuf;       /* Command arguments buffer */
  
  struct hid_func_descriptor hid_data;
};

static int apple_spi_hid_parse(struct hid_device *hid)
{
  struct apple_hid_kbd_data *applehidkbd = hid->driver_data;
  unsigned int rsize;
  char *rdesc;
  int ret;

  rsize = le16_to_cpu(applehidkbd->hid_data.report_desc_length);
  if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
    dbg_hid("weird size of report descriptor (%u)\n", rsize);
    return -EINVAL;
  }

  rdesc = kzalloc(rsize, GFP_KERNEL);
  if (!rdesc) {
    dbg_hid("couldn't allocate rdesc memory\n");
    return -ENOMEM;
  }

  memcpy(rdesc, applehidkbd->hid_data.report_desc, rsize);

  ret = hid_parse_report(hid, rdesc, rsize);
  kfree(rdesc);
  if (ret) {
    dbg_hid("parsing report descriptor failed\n");
    return ret;
  }

  return 0;
}

static int apple_spi_hid_get_report_length(struct hid_report *report)
{
  return ((report->size - 1) >> 3) + 1 +
    report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void apple_spi_hid_find_max_report(struct hid_device *hid, unsigned int type,
				    unsigned int *max)
{
  struct hid_report *report;
  unsigned int size;

  /* We should not rely on wMaxInputLength, as some devices may set it to
   * a wrong length. */
  list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
    size = apple_spi_hid_get_report_length(report);
    if (*max < size)
      *max = size;
  }
}

static void apple_spi_hid_free_buffers(struct apple_hid_kbd_data *applehidkbd)
{
  kfree(applehidkbd->inbuf);
  kfree(applehidkbd->rawbuf);
  kfree(applehidkbd->argsbuf);
  kfree(applehidkbd->cmdbuf);
  applehidkbd->inbuf = NULL;
  applehidkbd->rawbuf = NULL;
  applehidkbd->cmdbuf = NULL;
  applehidkbd->argsbuf = NULL;
  applehidkbd->bufsize = 0;
}

static int apple_spi_hid_alloc_buffers(struct apple_hid_kbd_data *applehidkbd, size_t report_size)
{
  /* the worst case is computed from the set_report command with a
   * reportID > 15 and the maximum report length */
  int args_len = sizeof(__u8) + /* ReportID */
    sizeof(__u8) + /* optional ReportID byte */
    sizeof(__u16) + /* data register */
    sizeof(__u16) + /* size of the report */
    report_size; /* report */

  applehidkbd->inbuf = kzalloc(report_size, GFP_KERNEL);
  applehidkbd->rawbuf = kzalloc(report_size, GFP_KERNEL);
  applehidkbd->argsbuf = kzalloc(args_len, GFP_KERNEL);
  applehidkbd->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

  if (!applehidkbd->inbuf || !applehidkbd->rawbuf || !applehidkbd->argsbuf || !applehidkbd->cmdbuf) {
    apple_spi_hid_free_buffers(applehidkbd);
    return -ENOMEM;
  }

  applehidkbd->bufsize = report_size;

  return 0;
}

static int apple_spi_hid_start(struct hid_device *hid)
{
  struct apple_hid_kbd_data *applehidkbd = hid->driver_data;
  int ret;
  unsigned int bufsize = HID_MIN_BUFFER_SIZE;

  apple_spi_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
  apple_spi_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
  apple_spi_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

  if (bufsize > applehidkbd->bufsize) {
    disable_irq(spi->irq);
    apple_spi_hid_free_buffers(applehidkbd);

    ret = apple_spi_hid_alloc_buffers(ihid, bufsize);
    enable_irq(client->irq);

    if (ret)
      return ret;
  }

  return 0;
}

static void apple_spi_hid_stop(struct hid_device *hid)
{
  hid->claimed = 0;
}

static int apple_spi_hid_open(struct hid_device *hid)
{
  struct apple_hid_kbd_data *applehidkbd = hid->driver_data;
  int ret = 0;

  ret = pm_runtime_get_sync(&applehidkbd->pdev->dev);
  if (ret < 0)
    return ret;

  set_bit(SPI_HID_STARTED, &applehidkbd->flags);
  return 0;
}

struct hid_ll_driver apple_hid_kbd_ll_driver = {
  .parse = apple_spi_hid_parse,
  .start = apple_spi_hid_start,
  .stop = apple_spi_hid_stop,
  .open = apple_spi_hid_open,
  .close = apple_spi_hid_close,
  .power = apple_spi_hid_power,
  .output_report = apple_spi_hid_output_report,
  .raw_request = apple_spi_hid_raw_request,
};
EXPORT_SYMBOL_GPL(apple_hid_kbd_ll_driver);

#ifdef CONFIG_ACPI
static const struct acpi_device_id spi_hid_acpi_match[] = {
  {"APP000D", 0 },
  { },
};
MODULE_DEVICE_TABLE(acpi, spi_hid_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id spi_hid_of_match[] = {
  { .compatible = "APPLE-SPI-TOPCASE" },
  {},
};
MODULE_DEVICE_TABLE(of, spi_hid_of_match);
#endif

static void apple_spi_hid_unregister_transport(void *data)
{
  /* TODO: implement me */
}

static int apple_spi_hid_probe(struct spi_device *spi)
{
  struct hid_device *hid;
  struct apple_hid_data *apple_hid;
  int ret;
  
  /*  */
  apple_hid = kzalloc(sizeof(struct apple_hid_data), GFP_KERNEL);
  if (!apple_hid)
    return -ENOMEM;

  pm_runtime_get_noresume(&spi->dev);
  pm_runtime_set_active(&spi->dev);
  pm_runtime_enable(&spi->dev);
  device_enable_async_suspend(&spi->dev);

  /* initialize interrupt */
  //  spi->irq = 0x0e; //TODO:JK: might be we should detect the interrupt
  //  shid->irq = -1;
	
  ret = apple_spi_hid_init_irq(spi);
  if (ret < 0)
    goto err_pm;
  

  hid = hid_allocate_device();
		
  if (IS_ERR(hid)) {
    ret = PTR_ERR(hid);
    goto err_hid_free;
  }

  apple_hid->hid = hid;
  hid->driver_data = apple_hid;

  hid->ll_driver = &apple_spi_hid_ll_driver;
  hid->dev.parent = &spi->dev;
  hid->bus = BUS_SPI;

  hid->version = le16_to_cpu(SPI_BCD_VERSION_APPLE);
  hid->vendor = le16_to_cpu(SPI_VENDOR_ID_APPLE);
  hid->product = le16_to_cpu(SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO);

  snprintf(hid->name, sizeof(hid->name), "%s %04hX:%04hX",
	   spi->modalias, hid->vendor, hid->product);
  strlcpy(hid->phys, dev_name(&spi->dev), sizeof(hid->phys));

  ret = hid_add_device(hid);
  if (ret) {
    if (ret != -ENODEV)
      hid_err(spi, "can't add hid device: %d\n", ret);
    goto err_hid_free;
  }

  dev_info(&spi->dev, "registered HID SPI driver\n");

  spi_set_drvdata(spi, hid);
  apple_hid->hid = hid;

  /* we need to allocate the command buffer without knowing the maximum
   * size of the reports. Let's use APPLE_SPI_HID_IOBUF_LENGTH, then we do the
   * real computation later. */
  ret = apple_spi_hid_alloc_buffers(hid, APPLE_SPI_HID_IOBUF_LENGTH);
  if (ret < 0)
    goto err_hid_free;

  ret = apple_spi_hid_fetch_hid_descriptor(apple_hid);
  if (ret != 0) {
    goto err_hid_free;
  }

  pm_runtime_put(&spi->dev);
	
  return 0;

 err_hid_free:
  free_irq(apple_hid->irq, apple_hid);
  hid_destroy_device(hid);

 err_pm:
  pm_runtime_put_noidle(&spi->dev);
  pm_runtime_disable(&spi->dev);

 err:
  apple_spi_hid_free_buffers(shid);
  kfree(shid);
  
  return ret;
}

static int apple_spi_hid_remove(struct spi_device *spi)
{
  struct hid_device *hid = spi_get_drvdata(spi);
  struct apple_hid_data *apple_hid = hid->driver_data;

  pm_runtime_get_sync(&spi->dev);
  pm_runtime_disable(&spi->dev);
  pm_runtime_set_suspended(&spi->dev);
  pm_runtime_put_noidle(&spi->dev);

  /* do not disturb */
  free_irq(apple_hid->irq, apple_hid);

  /* destroy hid */
  hid_destroy_device(hid);

  /* free buffers */
  apple_spi_hid_free_buffers(hid);
  
  kfree(apple_hid);
  
  return 0;
}

static void apple_spi_hid_shutdown(struct spi_device *spi)
{
  struct hid_device *hid = spi_get_drvdata(spi);
  struct apple_hid_data *apple_hid = hid->driver_data;
    
  /* power sleep */
  free_irq(apple_hid->irq, apple_hid);

  apple_spi_hid_set_power(his, APPLE_SPI_HID_PWR_SLEEP);
}

#ifdef CONFIG_PM_SLEEP
static int apple_spi_hid_suspend(struct device *dev)
{
  struct spi_device *spi = to_spi_device(dev);
  struct hid_device *hid = spi_get_drvdata(spi);
  int ret;

  if (hid->driver && hid->driver->suspend) {
    /*
     * Wake up the device so that IO issues in
     * HID driver's suspend code can succeed.
     */
    ret = pm_runtime_resume(dev);
    if (ret < 0)
      return -1;
    
    ret = hid->driver->suspend(hid, PMSG_SUSPEND);
    if (ret < 0)
      return -1;

    if (!pm_runtime_suspended(dev)) {
      /* Save some power */
      apple_spi_hid_set_power(hid, APPLE_SPI_HID_PWR_SLEEP);
    }
  }
  
  return 0;
}

static int apple_spi_hid_resume(struct device *dev)
{
  struct spi_device *spi = to_spi_device(dev);
  struct hid_device *hid = spi_get_drvdata(spi);
  int ret;
  
  /* We'll resume to full power */
  pm_runtime_disable(dev);
  pm_runtime_set_active(dev);
  pm_runtime_enable(dev);

  ret = apple_spi_hid_hwreset(hid);
  if (ret)
    return -1;

  if (hid->driver && hid->driver->reset_resume) {
    ret = hid->driver->reset_resume(hid);
    if (ret)
      return -1;
  }

  return 0;
}
#endif

#ifdef CONFIG_PM
static int apple_spi_hid_runtime_suspend(struct device *dev)
{
  struct spi_device *spi = to_spi_device(dev);
  struct hid_device *hid = spi_get_drvdata(spi);
	
  /* power sleep */
  apple_spi_hid_set_power(hid, APPLE_SPI_HID_PWR_SLEEP);

  return 0;
}

static int apple_spi_hid_runtime_resume(struct device *dev)
{
  struct spi_device *spi = to_spi_device(dev);
  struct hid_device *hid = spi_get_drvdata(spi);

  /* power on */
  apple_spi_hid_set_power(hid, APPLE_SPI_HID_PWR_ON);

  return 0;
}
#endif

static int apple_spi_probe(struct platform_device *pdev)
{
  struct spi_gpio *spi_gpio;

  applehidkbd->pdev = pdev;
  
  /* call SPI GPIO probe */
  spi_gpio_probe(pdev);
  spi_gpio = platform_get_drvdata(pdev);

  /* replace driver data */
  applehidkbd->spi_gpio = spi_gpio;
  platform_set_drvdata(pdev, applehidkbd);
  
  /* hid */
  hid = hid_allocate_device();
		
  if (IS_ERR(hid)) {
    result = PTR_ERR(hid);
    
    return result;
  }

  applehidkbd->hid = hid
  hid->driver_data = applehidkbd;

  hid->ll_driver = &spi_hid_ll_driver;
  hid->dev.parent = &spi->dev;
  hid->bus = BUS_SPI;

  hid->version = le16_to_cpu(SPI_BCD_VERSION_APPLE);
  hid->vendor = le16_to_cpu(SPI_VENDOR_ID_APPLE);
  hid->product = le16_to_cpu(SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO);

  snprintf(hid->name, sizeof(hid->name), "%s %04hX:%04hX",
	   spi->modalias, hid->vendor, hid->product);
  strlcpy(hid->phys, dev_name(&spi->dev), sizeof(hid->phys));

  ret = hid_add_device(hid);
  if (ret) {
    if (ret != -ENODEV)
      hid_err(spi, "can't add hid device: %d\n", ret);
    goto err_mem_free;
  }

  dev_info(&spi->dev, "registered HID keyboard driver\n");
  
  return(0);
}

static int apple_spi_remove(struct platform_device *pdev)
{
  spi_gpio_remove(pdev);
}

static struct apple_hid_data applehidkbd = {
  .hid_data = {
    .subclass = 0, /* No subclass */
    .protocol = 1, /* Keyboard */
    .report_length = APPLE_HID_KBD_REPORT_LENGTH,
    .report_desc_length = APPLE_HID_KBD_DESC_LENGTH,
    .report_desc[] = {
      0x05,0x01,
      0x09,0x06,
      0xa1,0x01,
      0x85,0x01,
      0x05,0x07,
      0x19,0xe0,
      0x29,0xe7,
      0x15,0x00,
      0x25,0x01,
      0x75,0x01,
      0x95,0x08,
      0x81,0x02,
      0x95,0x01,
      0x75,0x08,
      0x81,0x01,
      0x95,0x05,
      0x75,0x01,
      0x05,0x08,
      0x19,0x01,
      0x29,0x05,
      0x91,0x02,
      0x95,0x01,
      0x75,0x03,
      0x91,0x01,
      0x95,0x06,
      0x75,0x08,
      0x15,0x00,
      0x26,0xff,
      0x00,0x05,
      0x07,0x19,
      0x00,0x29,
      0xff,0x81,
      0x00,0x05,
      0x0c,0x75,
      0x01,0x95,
      0x01,0x09,
      0xb8,0x15,
      0x00,0x25,
      0x01,0x81,
      0x02,0x05,
      0xff,0x09,
      0x03,0x75,
      0x07,0x95,
      0x01,0x81,
      0x02,0xc0,
      0x05,0x0c,
      0x09,0x01,
      0xa1,0x01,
      0x85,0x52,
      0x15,0x00,
      0x25,0x01,
      0x75,0x01,
      0x95,0x01,
      0x09,0xcd,
      0x81,0x02,
      0x09,0xb3,
      0x81,0x02,
      0x09,0xb4,
      0x81,0x02,
      0x09,0xb5,
      0x81,0x02,
      0x09,0xb6,
      0x81,0x02,
      0x81,0x01,
      0x81,0x01,
      0x81,0x01,
      0x85,0x09,
      0x15,0x00,
      0x25,0x01,
      0x75,0x08,
      0x95,0x01,
      0x06,0x01,
      0xff,0x09,
      0x0b,0xb1,
      0x02,0x75,
      0x08,0x95,
      0x02,0xb1,
      0x01,0xc0,
      0x06,0x00,
      0xff,0x09,
      0x06,0xa1,
      0x01,0x06,
      0x00,0xff,
      0x09,0x06,
      0x15,0x00,
      0x26,0xff,
      0x00,0x75,
      0x08,0x95,
      0x40,0x85,
      0x3f,0x81,
      0x22,0xc0,
      0x06,0x00,
      0xff,0x09,
      0x0f,0xa1,
      0x01,0x06,
      0x00,0xff,
      0x09,0x0f,
      0x15,0x00,
      0x26,0xff,
      0x00,0x75,
      0x08,0x95,
      0x0f,0x85,
      0xbf,0x81,
      0x02,0xc0,    
    }
  };
};

static struct apple_hid_data applehidtp = {
  .hid_data = {
    .subclass = 0, /* No subclass */
    .protocol = 2, /* mouse */
    .report_length = APPLE_HID_TP_REPORT_LENGTH,
    .report_desc_length = APPLE_HID_TP_DESC_LENGTH,
    .report_desc[] = {
      0x05,0x01,
      0x09,0x02,
      0xa1,0x01,
      0x09,0x01,
      0xa1,0x00,
      0x05,0x09,
      0x19,0x01,
      0x29,0x03,
      0x15,0x00,
      0x25,0x01,
      0x85,0x02,
      0x95,0x03,
      0x75,0x01,
      0x81,0x02,
      0x95,0x01,
      0x75,0x05,
      0x81,0x01,
      0x05,0x01,
      0x09,0x30,
      0x09,0x31,
      0x15,0x81,
      0x25,0x7f,
      0x75,0x08,
      0x95,0x02,
      0x81,0x06,
      0x95,0x04,
      0x75,0x08,
      0x81,0x01,
      0xc0,0xc0,
      0x05,0x0d,
      0x09,0x05,
      0xa1,0x01,
      0x06,0x00,
      0xff,0x09,
      0x0c,0x15,
      0x00,0x26,
      0xff,0x00,
      0x75,0x08,
      0x95,0x10,
      0x85,0x3f,
      0x81,0x22,
      0xc0,0x06,
      0x00,0xff,
      0x09,0x0c,
      0xa1,0x01,
      0x06,0x00,
      0xff,0x09,
      0x0c,0x15,
      0x00,0x26,
      0xff,0x00,
      0x85,0x44,
      0x75,0x08,
      0x96,0x57,
      0x05,0x81,
      0x00,0xc0,
    }
  };
};

static struct spi_board_info applehid_spi_info[] = {
  {
    .modalias = "applespi:keyboard",
    .platform_data = &applehidkbd,
    .controller_data = (void *) gpioKeyboardMISO,
    .mode = SPI_MODE_0,
    .max_speed_hz = 47000,
    .irq = gpio_to_irq(gpioKeyboardMISO),
    .bus_num = 0,
    .chip_select = 4,
  },
  {
    .modalias = "applespi:trackpad",
    .platform_data = &applehidtp,
    .controller_data = (void *) gpioTrackpadMISO,
    .mode = SPI_MODE_0,
    .max_speed_hz = 47000,
    .irq = gpio_to_irq(gpioTrackpadMISO),
    .bus_num = 1,
    .chip_select = 4,
  },
};

struct spi_gpio_platform_data applehidkbd_pdata = {
  .sck = gpioKeyboardCLK,
  .mosi = gpioKeyboardMOSI,
  .miso = gpioKeyboardMISO,

  .num_chipselect = 4,
};

struct spi_gpio_platform_data applehidtp_pdata = {
  .sck = gpioTrackpadCLK,
  .mosi = gpioTrackpadMOSI,
  .miso = gpioTrackpadMISO,

  .num_chipselect = 4,
};

struct platform_device applehidkbd_pdev = {
  .name = "spi_gpio",
  .id = 0,
  .dev.platform = applehidkbd_pdata,
};

static const struct dev_pm_ops apple_spi_hid_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(apple_spi_hid_suspend, apple_spi_hid_resume)
	SET_RUNTIME_PM_OPS(apple_spi_hid_runtime_suspend, apple_spi_hid_runtime_resume,
			   NULL)
};

static const struct spi_device_id apple_spi_hid_id_table[] = {
	{ "hid", 0 },
	{ "hid-over-spi", 0 },
	{ "apple-spi-topcase", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, apple_spi_hid_id_table);

static struct spi_driver apple_spi_hid_driver = {
	.id_table = apple_spi_hid_id_table,
	
	.driver = {
		.name	= "apple_spi_hid",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
		.pm	= &apple_spi_hid_pm,
		.acpi_match_table = ACPI_PTR(apple_spi_hid_acpi_match),
		.of_match_table = of_match_ptr(apple_spi_hid_of_match),
	},

	.probe		= apple_spi_hid_probe,
	.remove		= apple_spi_hid_remove,
	.shutdown	= apple_spi_hid_shutdown,
	.id_table	= apple_spi_hid_id_table,
};

module_spi_driver(apple_spi_hid_driver);

MODULE_DESCRIPTION("Apple HID over SPI driver");
MODULE_AUTHOR("Joël Krähemann <jkraehemann@gmail.com>");
MODULE_LICENSE("GPL");
