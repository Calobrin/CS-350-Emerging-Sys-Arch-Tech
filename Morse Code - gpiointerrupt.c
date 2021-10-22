/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/Timer.h>
extern int usleep(useconds_t useconds);

//Boolean variable for toggling states of the SM
bool buttonSwitch = false;

//Function Declaration for Morse Code letters.
void MC_S() { //Turns the red LED on and off 3 times to represent "dot dot dot" part of the morse code letter S
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    usleep(1500000);
}
void MC_O() {//Turns the green LED on and off (longer for on) to represent the "dash dash dash" part of the morse code letter O
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(1500000);
}
void MC_K() {//Turns on the green LED once for the first "dash", then the red LED once briefly for the "dot", followed by another green LED "dash"
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    usleep(500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(1500000);
}

/*Enumerator state machine for "Morse Code States". MC is short for Morse Code
  We have MC_SOS state for the SOS morse code message, and also the MC_OK for the OK morse code message.
  Each state should be playing the morse code message over LED's by calling the letter functions.
  Also giving pause at the end of each word.
  Once the button has been pressed, it will swap the state over to play the other morse code message.
  And then reset the buttonSwitch boolean value to false so it can be checked if true again.
*/
enum MC_States { MC_SOS, MC_OK} MC_State = MC_SOS;

void MC_Blink() {

    switch(MC_State) {//State transitions and actions
    case MC_SOS:
        MC_S();
        MC_O();
        MC_S();
        usleep(350000);
        if (buttonSwitch == true) {
            MC_State = MC_OK;
            buttonSwitch = false;
        }
        break;
    case MC_OK:
        MC_O();
        MC_K();
        usleep(350000);
        if (buttonSwitch == true) {
            MC_State = MC_SOS;
            buttonSwitch = false;
        }
        break;
    default:
        MC_State = MC_SOS;
        break;
    }
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
MC_Blink(); //Calling state machine through the timer.
}
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 5000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;


    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
        }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
        }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    //Changes variable to true for state switching
    buttonSwitch = true;

}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    //Changes variable to true for state switching (I made it so both buttons do the same thing)
    buttonSwitch = true;
}

/*
 *  ======== mainThread ========
 */

void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();


    /* Configure the LED and button pins*/
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED*/
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback*/
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts*/
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
    */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin*/
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback*/
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initTimer(); //Call the timer which in turn calls the state machine.


    return (NULL);
}
