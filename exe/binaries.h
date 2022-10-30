#pragma once

#include "scancodes.h"
#include "advent.h"
//#include "zork.h"

static const struct embedded_file {
  const char *name;
  const unsigned char *data;
  size_t size;
  const char *description;
} embedded_files[] = {
   {"adventure",     adventure,      sizeof(adventure) - 1,      "Colossal Cave Adventure (1976 William Crowther, Don Woods)"},
// {"Zork",          zork,           sizeof(zork) - 1,           "Zork"},
};

static const uint8_t _scancode_map [] = {
    SC_A, SC_B, SC_C, SC_D, SC_E, 
    SC_F, SC_G, SC_H, SC_I, SC_J, 
    SC_K, SC_L, SC_M, SC_N, SC_O,
    SC_P, SC_Q, SC_R, SC_S, SC_T,
    SC_U, SC_V, SC_W, SC_X, SC_Y,
    SC_Z };

const char *find_embedded_file(const char *name, uint32_t *size) 
{
  const struct embedded_file *p;
  for (int i = 0; i < count_of(embedded_files); i++)
  {
    if (!strcmp(embedded_files[i].name, name)) 
    {
      if (size != NULL) 
      { 
        *size = p->size; 
      }
      return (const char *) embedded_files[i].data;
    }
  }
  return NULL;
}

void pal_loadbinary(const char *szFileName, uint8_t **dest, uint32_t *size)
{
    *dest =  (uint8_t *)find_embedded_file(szFileName, size);
}
