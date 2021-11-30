#ifndef _DS_DEF_H_
#define _DS_DEF_H_
// OneWire commands 

#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad 
#define COPYSCRATCH     0x48  // Copy EEPROM 
#define READSCRATCH     0xBE  // Read EEPROM 
#define WRITESCRATCH    0x4E  // Write to EEPROM 
#define RECALLSCRATCH   0xB8  // Reload from last known 
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power 
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition 
#define SKIPROM         0xCC

#endif