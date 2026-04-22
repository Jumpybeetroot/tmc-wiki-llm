# TMC4671 SPI High Speed 500ns Pause Constraint

**Type:** knowledge
**Summary:** TMC4671 requires a 500ns pause in Klipper's SPI handler (t_CSN_to_SCK) to run the SPI bus reliably above 2 MHz for AWD sync.
**Tags:** #engineering #hardware #spi #awd-sync
**Status:** draft
**Owner:** amcgregor
**Updated:** 2026-04-21

---

In relation to the AWD sync project, attempting to run the TMC4671 SPI bus at frequencies higher than 2 MHz reveals a timing constraint: we likely need to code a 500ns pause into Klipper's SPI implementation. 

This is because the TMC4671 requires a minimum `t_CSN_to_SCK` (time from Chip Select active to the first clock edge) of 500ns. When the SPI clock speed is increased beyond 2 MHz, the hardware SPI peripherals can start the clock too quickly after asserting CS, violating the TMC4671's timing requirements. To bypass this, Klipper's lower-level CS assertion routines may need to be modified or padded with a hardware delay.
