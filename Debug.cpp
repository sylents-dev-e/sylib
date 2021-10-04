#include "Debug.h"
#include "BufferedSerial.h"
#include <stdio.h>
#include <string.h>



Debug::Debug(BufferedSerial* pserial)
 : m_pSerial(pserial)
 {
     // set baudrate
     m_pSerial->set_baud(115600);
     // opens file descriptor for reading and writing "r+"
     m_pFile = fdopen(m_pSerial,"r+");
 }

 void Debug::debug(const char * dbg)
 {
     fprintf(m_pFile, dbg);
     //m_pSerial->write(dbg, strlen(dbg));
 }