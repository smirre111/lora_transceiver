

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>


#define SS 18
#define RST 14
#define DI0 26
#define BAND 433E6 //915E6

LoRaClass LoRa;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//LoRaClass LoRa;

/*
 * This example shows how to use interrupts to catch various LoRa events:
 *  - RxDone
 *  - TxDone
 *  - CADDone
 *  - RxTimeout
 *  - FHSSChangeChannel
 *  - CADDetected
 *  - ValidHeader
 *  - CRCError
 *  - PLLLock
 *  - ModeReady
 *  - ClockOutput
 *
 * This specific example goes through the process of turning on CAD detection and
 * when a successful CAD was observed, put the LoRa chip to receive mode.
 */

/*
 * It's possible to handle LoRa CAD (channel activity detection, page 43) the following way via interrupts:
 *
 * 1. Write your interrupt service routine (ISR) which is executed when an IRQ arrives to the pin which is connected to DIO[0-5].
 * 2. Configure ISR in Arduino for LoRa DIO[0-5] pin: LoRa.setInterruptMode(0, LORA_IRQ_MODE_CADDONE)
 * 3. Start CAD: LoRa.cad()
 * 4. Check if valid CAD detected after an IRQ arrived : LoRa.readInterrupts() & LORA_IRQ_FLAG_CAD_DETECTED
 * 5. Clear IRQs to be able to restart CAD process later: LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
 *
 * You may use all five DIOs concurrently connected to different Arduino pins (though those pins should intercept IRQs).
 *
 * Wiring of the current example: LoRa DIO0 - Arduino D3
 */

/* Copyright © 2018 Szőts Ákos <szotsaki@gmail.com> */

// Interrupt Service Routine (ISR)
// Must be as short as physically possible - assigning a variable should make it
static volatile bool interruptHappened = false;

void IRAM_ATTR isr_pinD3()
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptHappened = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;

  SPI.begin(5, 19, 27, SS);

  LoRa.setPins(SS,RST,DI0);

  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed");
    while (1)
      ;
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSyncWord(0x12);

  // For other constants, like FHSS change channel, CRC error, or RX timeout, see the LoRa.h header file.
  // Choose from LORA_IRQ_DIOx_ variants and use this "x" number in place of the first parameter.
  // Not all DIOx and interrupt type mixes are possible.
  LoRa.setInterruptMode(0 /* DIO0 */, LORA_IRQ_DIO0_CADDONE);

  // Launch our ISR function when LoRa interrupt happens
  attachInterrupt(digitalPinToInterrupt(DI0), isr_pinD3, RISING);

  // Start LoRa CAD process
  LoRa.cad();

  Serial.println("Starting LoRa succeeded");
}

void loop()
{
  if (interruptHappened)
  {
    portENTER_CRITICAL(&timerMux);
    interruptHappened = false;
    portEXIT_CRITICAL(&timerMux);

    const uint8_t loraInterrupts = LoRa.readInterrupts();
    if (loraInterrupts & LORA_IRQ_FLAG_CAD_DETECTED)
    {                     // Here use LORA_IRQ_FLAG_* variants from LoRa.h
      LoRa.parsePacket(); // Put into RXSINGLE mode
                          // ... Process packet if there's one
    }

    // Do not let the ISR function and the loop() write and read the interruptHappened variable at the same time
    //ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      // It's possible that the ISR function had set interruptHappened to "true" while we were receiving packets.
      // Check again to be sure that we clear interrupt flags only when we received no further IRQs.
      if (!interruptHappened)
      {
        LoRa.clearInterrupts(LORA_IRQ_FLAG_CAD_DETECTED | LORA_IRQ_FLAG_CAD_DONE);
      }
    }

    // Restart CAD process
    LoRa.cad();
  }
}
