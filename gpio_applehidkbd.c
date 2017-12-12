#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/time.h>       // Using the clock to measure time between button presses
#define  DEBOUNCE_TIME 200    ///< The default bounce time -- 200ms

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joel Krähemann");
MODULE_DESCRIPTION("Apple HID Keyboard driver over GPIO");
MODULE_VERSION("0.1");

static bool isRising = 1;                   ///< Rising edge is the default IRQ property
module_param(isRising, bool, S_IRUGO);      ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(isRising, " Rising edge = 1 (default), Falling edge = 0");  ///< parameter description

static unsigned int gpioKeyboardCS_L = 83;
module_param(gpioKeyboard, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboard, " GPIO Keyboard CS_L number (default=83)");  ///< parameter description

static unsigned int gpioKeyboardCLK = 84;
module_param(gpioKeyboard, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboard, " GPIO Keyboard CLK number (default=84)");  ///< parameter description

static unsigned int gpioKeyboardMISO = 85;
module_param(gpioKeyboard, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboard, " GPIO Keyboard MISO number (default=85)");  ///< parameter description

static unsigned int gpioKeyboardMOSI = 86;
module_param(gpioKeyboard, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioKeyboard, " GPIO Keyboard MOSI number (default=86)");  ///< parameter description

static char   gpioName[8] = "gpioXXX";      ///< Null terminated default string -- just in case
static int    irqNumber;                    ///< Used to share the IRQ number within this file
static struct timespec ts_last, ts_current, ts_diff;  ///< timespecs from linux/time.h (has nano precision)

/* hid descriptor for apple keyboard */
tatic struct hidg_func_descriptor apple_kbd_hid_data = {
  .subclass		= 0, /* No subclass */
  .protocol		= 1, /* Keyboard */
  .report_length	= 8,
  .report_desc_length	= 208,
  .report_desc		= {
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

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  applegpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

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
static irq_handler_t applegpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
  //TODO:JK: MOSI
  
  getnstimeofday(&ts_current);         // Get the current time as ts_current
  ts_diff = timespec_sub(ts_current, ts_last);   // Determine the time difference between last 2 presses
  ts_last = ts_current;                // Store the current time as the last time ts_last
  
  return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

// This next calls are  mandatory -- they identify the initialization function
// and the cleanup function (as above).
module_init(appleHIDKeyboard_init);
module_exit(appleHIDKeyboard_exit);
