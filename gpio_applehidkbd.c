#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_gpio.h>
#define  DEBOUNCE_TIME 200    ///< The default bounce time -- 200ms

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joel Krähemann");
MODULE_DESCRIPTION("Apple HID Keyboard driver over GPIO");
MODULE_VERSION("0.1");

#define DRIVER_NAME     "applespi:keyboard"
#define SPI_MISO_GPIO   85
#define SPI_MOSI_GPIO   86
#define SPI_SCK_GPIO    84
#define SPI_N_CHIPSEL   4
#include "spi-gpio.c"

static bool isRising = 1;                   ///< Rising edge is the default IRQ property
module_param(isRising, bool, S_IRUGO);      ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(isRising, " Rising edge = 1 (default), Falling edge = 0");  ///< parameter description

static unsigned int gpioKeyboardCS_L = 83;
module_param(gpioKeyboardCS_L, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboardCS_L, " GPIO Keyboard CS_L number (default=83)");  ///< parameter description

static unsigned int gpioKeyboardCLK = 84;
module_param(gpioKeyboardCLK, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboardCLK, " GPIO Keyboard CLK number (default=84)");  ///< parameter description

static unsigned int gpioKeyboardMISO = 85;
module_param(gpioKeyboardMISO, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboardMISO, " GPIO Keyboard MISO number (default=85)");  ///< parameter description

static unsigned int gpioKeyboardMOSI = 86;
module_param(gpioKeyboardMOSI, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboardMOSI, " GPIO Keyboard MOSI number (default=86)");  ///< parameter description

static char   gpioName[8] = "gpioXXX";      ///< Null terminated default string -- just in case
static int    irqNumber;                    ///< Used to share the IRQ number within this file
static struct timespec ts_last, ts_current, ts_diff;  ///< timespecs from linux/time.h (has nano precision)

/* hid descriptor for apple keyboard */

#define SPI_VENDOR_ID_APPLE (0x05ac)
#define SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO (0x0276)
#define SPI_BCD_VERSION_APPLE (3)

#define APPLE_HID_KBD_REPORT_LENGTH (8)
#define APPLE_HID_KBD_DESC_LENGTH (208)

static struct spi_gpio_platform_data pdata = {

};

static struct platform_device pdevice = {
  .name = "spi_gpio",
  .id = 0x0,
  .dev.platform = pdata,
};

static struct apple_hid_kbd_data {
  struct spi_device *spi;
  acpi_handle handle;

  u64 spi_sclk_period;	/* period in ns */
  u64 spi_cs_delay;    	/* cs-to-clk delay in us */

  
  u8 rx_buffer[APPLE_HID_KBD_REPORT_LENGTH];
  u8 tx_status[APPLE_HID_KBD_DESC_LENGTH];

  acpi_handle sien;
  acpi_handle sist;

  struct hid_device *hid;

  unsigned short bufsize;
  unsigned char *inbuf;         /* Input buffer */
  unsigned char *rawbuf;        /* Raw Input buffer */
  unsigned char *cmdbuf;        /* Command buffer */
  unsigned char *argsbuf;       /* Command arguments buffer */
  
  static const struct hid_func_descriptor hid_data = {
    unsigned char subclass = 0; /* No subclass */
    unsigned char protocol = 1; /* Keyboard */
    unsigned short report_length = APPLE_HID_KBD_REPORT_LENGTH;
    unsigned short report_desc_length = APPLE_HID_KBD_DESC_LENGTH;
    unsigned char report_desc[] = {
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

static int appleHIDKeyboard_hid_parse(struct hid_device *hid)
{
  struct spi_device *spi = hid->driver_data;
  struct apple_hid_kbd_data *applehidkbd = spi_get_drvdata(spi);
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

static int appleHIDKeyboard_hid_get_report_length(struct hid_report *report)
{
  return ((report->size - 1) >> 3) + 1 +
    report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void appleHIDKeyboard_hid_find_max_report(struct hid_device *hid, unsigned int type,
				    unsigned int *max)
{
  struct hid_report *report;
  unsigned int size;

  /* We should not rely on wMaxInputLength, as some devices may set it to
   * a wrong length. */
  list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
    size = appleHIDKeyboard_hid_get_report_length(report);
    if (*max < size)
      *max = size;
  }
}

static void appleHIDKeyboard_hid_free_buffers(struct apple_hid_kbd_data *applehidkbd)
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

static int appleHIDKeyboard_hid_alloc_buffers(struct apple_hid_kbd_data *applehidkbd, size_t report_size)
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
    i2c_hid_free_buffers(applehidkbd);
    return -ENOMEM;
  }

  applehidkbd->bufsize = report_size;

  return 0;
}

static int appleHIDKeyboard_hid_start(struct hid_device *hid)
{
  struct spi_device *spi = hid->driver_data;
  struct apple_hid_kbd_data *applehidkbd = spi_get_drvdata(spi);
  int ret;
  unsigned int bufsize = HID_MIN_BUFFER_SIZE;

  appleHIDKeyboard_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
  appleHIDKeyboard_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
  appleHIDKeyboard_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

  if (bufsize > applehidkbd->bufsize) {
    disable_irq(spi->irq);
    appleHIDKeyboard_hid_free_buffers(applehidkbd);

    ret = appleHIDKeyboard_hid_alloc_buffers(ihid, bufsize);
    enable_irq(client->irq);

    if (ret)
      return ret;
  }

  return 0;
}

static void appleHIDKeyboard_hid_stop(struct hid_device *hid)
{
  hid->claimed = 0;
}

struct hid_ll_driver apple_hid_kbd_ll_driver = {
  .parse = appleHIDKeyboard_hid_parse,
  .start = appleHIDKeyboard_hid_start,
  .stop = appleHIDKeyboard_hid_stop,
  .open = appleHIDKeyboard_hid_open,
  .close = appleHIDKeyboard_hid_close,
  .power = appleHIDKeyboard_hid_power,
  .output_report = appleHIDKeyboard_hid_output_report,
  .raw_request = appleHIDKeyboard_hid_raw_request,
};
EXPORT_SYMBOL_GPL(apple_hid_kbd_ll_driver);

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t appleHIDKeyboard_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static struct kobject *apple_kobj;

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init appleHIDKeyboard_init(void){
   int result = 0;
   unsigned long IRQflags = IRQF_TRIGGER_RISING;      // The default is a rising-edge interrupt

   printk(KERN_INFO "Apple HID Keyboard: Initializing the Apple HID CS_L LKM\n");
   sprintf(gpioName, "gpio%d", gpioKeyboardCS_L);     // Create the gpio115 name for /sys/ebb/gpio115

   // create the kobject sysfs entry at /sys/ebb -- probably not an ideal location!
   ebb_kobj = kobject_create_and_add("appleHIDKeyboard", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
   if(!ebb_kobj){
      printk(KERN_ALERT "Apple HID Keyboard: failed to create kobject mapping\n");
      return -ENOMEM;
   }

   getnstimeofday(&ts_last);                          // set the last time to be the current time
   ts_diff = timespec_sub(ts_last, ts_last);          // set the initial time difference to be 0

   gpio_request(gpioKeyboardCS_L, "sysfs");           // gpioKeyboardCS_L is hardcoded to 87, request it
   gpio_direction_output(gpioKeyboardCS_L, 0);        // Set the gpio to be in output mode and 0
   gpio_export(gpioKeyboardCS_L, false);              // Causes gpio87 to appear in /sys/class/gpio
   
   gpio_request(gpioKeyboardCLK, "sysfs");            // gpioKeyboardCLK is hardcoded to 88, request it
   gpio_direction_output(gpioKeyboardCLK, 0);         // Set the gpio to be in output mode and 0
   gpio_export(gpioKeyboardCLK, false);               // Causes gpio88 to appear in /sys/class/gpio

   gpio_request(gpioKeyboardMISO, "sysfs");           // gpioKeyboardMISO is hardcoded to 89, request it
   gpio_direction_input(gpioKeyboardMISO);            // Set the gpio to be in input mode
   gpio_export(gpioKeyboardMISO, false);              // Causes gpio89 to appear in /sys/class/gpio

   gpio_request(gpioTrackpapMOSI, "sysfs");           // gpioKeyboardMOSI is hardcoded to 90, request it
   gpio_direction_output(gpioKeyboardMOSI, 0);        // Set the gpio to be in output mode and 0
   gpio_export(gpioKeyboardMOSI, false);              // Causes gpio90 to appear in /sys/class/gpio

   /// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   irqNumber = gpio_to_irq(gpioKeyboardMISO);
   printk(KERN_INFO "Apple HID Keyboard MISO: The keyboard is mapped to IRQ: %d\n", irqNumber);

   if(!isRising){                           // If the kernel parameter isRising=0 is supplied
      IRQflags = IRQF_TRIGGER_FALLING;      // Set the interrupt to be on the falling edge
   }
   // This next call requests an interrupt line
   result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) applegpio_irq_handler, // The pointer to the handler function below
                        IRQflags,              // Use the custom kernel param to set interrupt type
                        "apple_hid_keyboard",  // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay
   return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit appleHIDKeyboard_exit(void){
   kobject_put(apple_kobj);                 // clean up -- remove the kobject sysfs entry
   gpio_unexport(gpioKeyboardCS_L);         // Unexport the KeyboardCS_L GPIO
   free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   gpio_unexport(gpioKeyboardCLK);          // Unexport the KeyboardCLK GPIO
   gpio_unexport(gpioKeyboardMISO);         // Unexport the KeyboardMISO GPIO
   gpio_unexport(gpioKeyboardMOSI);         // Unexport the KeyboardMOSI GPIO
   gpio_free(gpioKeyboardCS_L);             // Free the KeyboardCS_L GPIO
   gpio_free(gpioKeyboardCLK);              // Free the KeyboardCLK GPIO
   gpio_free(gpioKeyboardMISO);             // Free the KeyboardMISO GPIO
   gpio_free(gpioKeyboardMOSI);             // Free the KeyboardMOSI GPIO
   printk(KERN_INFO "Apple HID Keyboard: Goodbye from the GPIO driver!\n");
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t appleHIDKeyboard_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
  //TODO:JK: MOSI
  
  getnstimeofday(&ts_current);         // Get the current time as ts_current
  ts_diff = timespec_sub(ts_current, ts_last);   // Determine the time difference between last 2 presses
  ts_last = ts_current;                // Store the current time as the last time ts_last
  
  return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

static int appleHIDKeyboard_setup_spi(struct applespi_data *applehidkbd)
{
  return 0;
}

static int appleHIDKeyboard_enable_spi(struct applespi_data *applehidkbd)
{
  int result;
  long long unsigned int spi_status;

  /* Check if SPI is already enabled, so we can skip the delay below */
  result = acpi_evaluate_integer(applehidkbd->sist, NULL, NULL, &spi_status);
  if (ACPI_SUCCESS(result) && spi_status)
    return 0;

  /* SIEN(1) will enable SPI communication */
  result = acpi_execute_simple_method(applehidkbd->sien, NULL, 1);
  if (ACPI_FAILURE(result)) {
    pr_err("SIEN failed: %s\n", acpi_format_exception(result));
    return -ENODEV;
  }

  /*
   * Allow the SPI interface to come up before returning. Without this
   * delay, the SPI commands to enable multitouch mode may not reach
   * the trackpad controller, causing pointer movement to break upon
   * resume from sleep.
   */
  msleep(50);

  return 0;
}

static int appleHIDKeyboard_probe(struct spi_device *spi)
{
  struct apple_hid_kbd_data *applehidkbd;
  struct hid_device *hid;
  
  /* Allocate driver data */
  applehidkbd = devm_kzalloc(&spi->dev, sizeof(*applehidkbd), GFP_KERNEL);
  if (!applehidkbd)
    return -ENOMEM;

  applehidkbd->spi = spi;
  applehidkbd->handle = ACPI_HANDLE(&spi->dev);
  
  /* Store the driver data */
  spi_set_drvdata(spi, applehidkbd);
  
  /* setup SPI GPIO */
  
  /* Cache ACPI method handles */
  if (ACPI_FAILURE(acpi_get_handle(applehidkbd->handle, "SIEN",
				   &applehidkbd->sien)) ||
      ACPI_FAILURE(acpi_get_handle(applehidkbd->handle, "SIST",
				   &applehidkbd->sist))) {
    pr_err("Failed to get required ACPI method handle\n");
    return -ENODEV;
  }

  /* Switch on the SPI interface */
  result = appleHIDKeyboard_setup_spi(applehidkbd);
  if (result)
    return result;

  result = appleHIDKeyboard_enable_spi(applehidkbd);
  if (result)
    return result;

  /* hid */
  hid = hid_allocate_device();
		
  if (IS_ERR(hid)) {
    result = PTR_ERR(hid);
    
    return result;
  }

  applehidkbd->hid = hid
  hid->driver_data = spi;

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

